"""One-step learned RGB-D action modifier.

The implementation preserves the inherited VAE -> MDN-RNN -> CBF/CLF-like
pipeline. Its output is explicitly tagged as empirical, not formal proof.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np
import torch
import torch.nn.functional as functional

from cbf_rover.domain import ActionModificationResult, CertificateEvaluation

from .certificates import CBFNetwork, CLFNetwork
from .checkpoints import (
    LearnedRGBDCheckpointSet,
    load_checkpoint,
    state_dict_from_checkpoint,
)
from .claims import LEARNED_RGBD_CLAIM
from .config import LearnedRGBDSafetyConfig
from .mdrnn import MDNRNN, expected_latent
from .preprocess import RGBDPreprocessConfig, preprocess_rgbd_tensors
from .vae import RGBDVAE, RGBDVAEConfig


SafetyInfo = dict[str, bool | float | str]
HiddenState = tuple[torch.Tensor, torch.Tensor]


class LearnedRGBDActionModifier:
    """Stateful empirical modifier for a two-dimensional normalized action."""

    name = "learned_rgbd_world_model_modifier"

    def __init__(
        self,
        *,
        checkpoints: LearnedRGBDCheckpointSet,
        device: torch.device | str,
        config: LearnedRGBDSafetyConfig | None = None,
    ) -> None:
        if isinstance(device, str) and device.lower() == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.config = config or LearnedRGBDSafetyConfig()
        self.preprocess_config = RGBDPreprocessConfig(
            image_size=self.config.image_size,
            min_depth_m=self.config.min_depth_m,
            max_depth_m=self.config.max_depth_m,
            input_is_bgr=self.config.input_is_bgr,
        )
        checkpoints.validate()

        self.vae = RGBDVAE(RGBDVAEConfig(latent_dim=self.config.latent_size)).to(
            self.device
        )
        self.vae.load_state_dict(
            state_dict_from_checkpoint(load_checkpoint(checkpoints.vae, self.device))
        )
        self.vae.eval()

        self.mdrnn = MDNRNN(
            latent_dim=self.config.latent_size,
            action_dim=self.config.action_size,
            hidden_dim=self.config.recurrent_hidden_size,
            num_gaussians=self.config.gaussian_components,
        ).to(self.device)
        self.mdrnn.load_state_dict(
            state_dict_from_checkpoint(load_checkpoint(checkpoints.mdrnn, self.device))
        )
        # Evaluation mode remains differentiable with respect to the candidate
        # action while keeping inference deterministic.
        self.mdrnn.eval()

        self.cbf = CBFNetwork(input_dim=self.config.latent_size).to(self.device)
        self.cbf.load_state_dict(
            state_dict_from_checkpoint(load_checkpoint(checkpoints.cbf, self.device))
        )
        self.cbf.eval()

        self.clf = CLFNetwork(input_dim=self.config.latent_size).to(self.device)
        self.clf.load_state_dict(
            state_dict_from_checkpoint(load_checkpoint(checkpoints.clf, self.device))
        )
        self.clf.eval()

        for model in (self.vae, self.mdrnn, self.cbf, self.clf):
            for parameter in model.parameters():
                parameter.requires_grad = False

        self.hidden = self.mdrnn.rnn.init_hidden(1, self.device)

    def reset_episode(self, *, seed: int | None = None) -> None:
        """Clear recurrent state at an experiment or simulator reset."""

        del seed
        self.hidden = self.mdrnn.rnn.init_hidden(1, self.device)

    def encode(self, rgb: np.ndarray, depth: np.ndarray) -> torch.Tensor:
        """Encode one aligned RGB-D observation to its deterministic mean."""

        rgb_tensor, depth_tensor = preprocess_rgbd_tensors(
            rgb,
            depth,
            self.preprocess_config,
            self.device,
        )
        with torch.no_grad():
            return self.vae.encode_to_latent(rgb_tensor, depth_tensor, use_mu=True)

    def predict_next(
        self,
        latent: torch.Tensor,
        action: np.ndarray,
        hidden: HiddenState,
    ) -> tuple[torch.Tensor, HiddenState]:
        action_tensor = (
            torch.as_tensor(action, dtype=torch.float32, device=self.device)
            .view(1, 1, self.config.action_size)
            .clamp(0.0, 1.0)
        )
        weights, means, _scales, next_hidden = self.mdrnn(
            latent.unsqueeze(0),
            action_tensor,
            hidden=hidden,
        )
        return expected_latent(weights, means).squeeze(1), next_hidden

    def _predict_next_tensor(
        self,
        latent: torch.Tensor,
        action: torch.Tensor,
        hidden: HiddenState,
    ) -> tuple[torch.Tensor, HiddenState]:
        action_sequence = action.view(1, 1, self.config.action_size).clamp(0.0, 1.0)
        weights, means, _scales, next_hidden = self.mdrnn(
            latent.unsqueeze(0),
            action_sequence,
            hidden=hidden,
        )
        return expected_latent(weights, means).squeeze(1), next_hidden

    def _penalty(
        self,
        clf_now: torch.Tensor,
        cbf_now: torch.Tensor,
        clf_next: torch.Tensor,
        cbf_next: torch.Tensor,
    ) -> tuple[torch.Tensor, SafetyInfo]:
        if not all(
            bool(torch.isfinite(value).all().item())
            for value in (clf_now, cbf_now, clf_next, cbf_next)
        ):
            raise FloatingPointError(
                "learned RGB-D model produced a non-finite certificate score"
            )
        cbf_value = float(cbf_now.detach().item())
        if cbf_value < self.config.safe_cbf_margin:
            safety_state = "safe"
        elif cbf_value < 0.0:
            safety_state = "critical"
        else:
            safety_state = "danger"

        clf_violation = functional.relu(clf_next - self.config.clf_decay * clf_now)
        if safety_state == "danger":
            target = torch.full_like(cbf_next, self.config.cbf_recovery_target)
            cbf_violation = functional.relu(cbf_next - target)
            penalty = cbf_violation.mean()
        else:
            cbf_violation = functional.relu(
                cbf_next - self.config.cbf_decay * cbf_now
            )
            penalty = cbf_violation.mean()
            if safety_state == "safe":
                penalty = penalty + clf_violation.mean()
        if not bool(torch.isfinite(penalty).all().item()):
            raise FloatingPointError(
                "learned RGB-D model produced a non-finite penalty"
            )

        info: SafetyInfo = {
            "safety_state": safety_state,
            "clf_now": float(clf_now.detach().item()),
            "clf_next": float(clf_next.detach().item()),
            "cbf_now": cbf_value,
            "cbf_next": float(cbf_next.detach().item()),
            "clf_violation": float(clf_violation.detach().item()),
            "cbf_violation": float(cbf_violation.detach().item()),
            "penalty": float(penalty.detach().item()),
        }
        return penalty, info

    def evaluate_action(
        self,
        latent: torch.Tensor,
        action: np.ndarray,
    ) -> tuple[bool, SafetyInfo]:
        """Evaluate the inherited one-step learned CBF/CLF-like condition."""

        with torch.no_grad():
            clf_now = self.clf(latent)
            cbf_now = self.cbf(latent)
            next_latent, _ = self.predict_next(latent, action, self.hidden)
            clf_next = self.clf(next_latent)
            cbf_next = self.cbf(next_latent)
            penalty, info = self._penalty(clf_now, cbf_now, clf_next, cbf_next)
        return bool(penalty.item() <= self.config.satisfaction_tolerance), info

    def optimize(
        self,
        latent: torch.Tensor,
        action: np.ndarray,
    ) -> tuple[np.ndarray, SafetyInfo]:
        """Apply the inherited short-horizon Adam action correction."""

        latent = latent.detach()
        reference = torch.as_tensor(
            action,
            dtype=torch.float32,
            device=self.device,
        ).view(1, self.config.action_size)
        candidate = reference.clone().detach().requires_grad_(True)
        optimizer = torch.optim.Adam(
            [candidate],
            lr=self.config.optimization_learning_rate,
        )

        if self.config.max_action_delta > 0.0:
            lower = (reference - self.config.max_action_delta).clamp(0.0, 1.0)
            upper = (reference + self.config.max_action_delta).clamp(0.0, 1.0)
        else:
            lower = torch.zeros_like(reference)
            upper = torch.ones_like(reference)

        with torch.no_grad():
            clf_now = self.clf(latent)
            cbf_now = self.cbf(latent)

        best_action = reference.detach().clone()
        _reference_safe, best_info = self.evaluate_action(
            latent,
            reference.squeeze(0).detach().cpu().numpy(),
        )
        best_penalty = float(best_info["penalty"])
        for step in range(self.config.optimization_steps):
            optimizer.zero_grad()
            hidden = (
                self.hidden[0].detach().clone(),
                self.hidden[1].detach().clone(),
            )
            next_latent, _ = self._predict_next_tensor(latent, candidate, hidden)
            penalty, _pre_step_info = self._penalty(
                clf_now,
                cbf_now,
                self.clf(next_latent),
                self.cbf(next_latent),
            )
            bounded = torch.max(torch.min(candidate.clamp(0.0, 1.0), upper), lower)
            action_loss = functional.mse_loss(bounded, reference)
            loss = (
                self.config.action_weight * action_loss
                + self.config.safety_weight * penalty
            )
            if not bool(torch.isfinite(loss).all().item()):
                raise FloatingPointError(
                    "learned RGB-D action optimization produced a non-finite loss"
                )
            loss.backward()
            optimizer.step()
            with torch.no_grad():
                candidate.clamp_(0.0, 1.0)
                candidate.copy_(torch.max(torch.min(candidate, upper), lower))
                hidden = (
                    self.hidden[0].detach().clone(),
                    self.hidden[1].detach().clone(),
                )
                next_latent, _ = self._predict_next_tensor(
                    latent,
                    candidate,
                    hidden,
                )
                post_step_penalty, post_step_info = self._penalty(
                    clf_now,
                    cbf_now,
                    self.clf(next_latent),
                    self.cbf(next_latent),
                )
            post_step_value = float(post_step_penalty.item())
            if post_step_value < best_penalty:
                best_penalty = post_step_value
                best_action = candidate.detach().clone()
                best_info = post_step_info | {
                    "optimization_steps": float(step + 1)
                }
            if post_step_value <= self.config.satisfaction_tolerance:
                break

        return (
            best_action.squeeze(0).detach().cpu().numpy().astype(np.float32),
            best_info,
        )

    def modify(
        self,
        reference_action: np.ndarray,
        state: torch.Tensor,
        context: Mapping[str, Any] | None = None,
    ) -> ActionModificationResult[np.ndarray]:
        """Evaluate and, only when necessary, correct one action."""

        del context
        reference = np.asarray(reference_action, dtype=np.float32)
        if reference.shape != (self.config.action_size,):
            raise ValueError(
                f"reference_action must have shape ({self.config.action_size},)"
            )
        if not np.isfinite(reference).all():
            raise FloatingPointError("reference_action must contain only finite values")
        if np.any(reference < 0.0) or np.any(reference > 1.0):
            raise ValueError("reference_action must stay inside normalized bounds [0, 1]")
        if not isinstance(state, torch.Tensor):
            raise TypeError("state must be a torch.Tensor")
        if state.shape != (1, self.config.latent_size):
            raise ValueError(
                f"state must have shape (1, {self.config.latent_size})"
            )
        if not bool(torch.isfinite(state).all().item()):
            raise FloatingPointError("latent state must contain only finite values")

        is_safe, info = self.evaluate_action(state, reference)
        modified = reference.copy()
        if not is_safe:
            modified, _optimization_info = self.optimize(state, reference)
            final_safe, info = self.evaluate_action(state, modified)
        else:
            final_safe = True

        intervention_norm = float(np.linalg.norm(modified - reference))
        penalty = max(float(info["penalty"]), 0.0)
        info = info | {"initial_safe": is_safe, "final_safe": final_safe}
        certificate = CertificateEvaluation(
            name="learned_rgbd_one_step_condition",
            value=penalty,
            satisfied=penalty <= self.config.satisfaction_tolerance,
            violation=penalty,
            claim=LEARNED_RGBD_CLAIM,
            metadata=info,
        )
        return ActionModificationResult(
            reference_action=reference,
            modified_action=modified,
            intervened=intervention_norm > 0.0,
            intervention_norm=intervention_norm,
            feasible=final_safe,
            certificates=(certificate,),
            metadata=info,
        )

    def update_hidden(self, latent: torch.Tensor, action: np.ndarray) -> None:
        """Advance recurrent state using the action actually executed."""

        executed = np.asarray(action, dtype=np.float32)
        if executed.shape != (self.config.action_size,):
            raise ValueError(
                f"executed action must have shape ({self.config.action_size},)"
            )
        if not np.isfinite(executed).all():
            raise FloatingPointError("executed action must contain only finite values")
        if np.any(executed < 0.0) or np.any(executed > 1.0):
            raise ValueError("executed action must stay inside normalized bounds [0, 1]")
        with torch.no_grad():
            _, self.hidden = self.predict_next(latent, executed, self.hidden)

    def update_after_step(
        self,
        state: torch.Tensor,
        executed_action: np.ndarray,
        context: Mapping[str, Any] | None = None,
    ) -> None:
        """Generic runner hook that advances the recurrent hidden state once."""

        del context
        self.update_hidden(state, executed_action)
