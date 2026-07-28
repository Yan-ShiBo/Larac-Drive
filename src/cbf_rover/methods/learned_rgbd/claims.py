"""Evidence statement specific to the learned RGB-D baseline."""

from cbf_rover.core import ClaimKind, SafetyClaim


LEARNED_RGBD_CLAIM = SafetyClaim(
    kind=ClaimKind.EMPIRICAL,
    statement=(
        "The learned RGB-D scores and latent prediction are empirical "
        "approximations and do not constitute a formal safety proof."
    ),
    assumptions=(
        "Runtime observations resemble the training distribution.",
        "Checkpoint architecture and preprocessing configuration match.",
    ),
)

__all__ = ["LEARNED_RGBD_CLAIM"]
