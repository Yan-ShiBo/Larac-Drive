"""Server-side command line for the Rover-first research chain."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="cbf-rover-research")
    commands = parser.add_subparsers(dest="command", required=True)

    train = commands.add_parser("train-sac")
    train.add_argument("--seed", type=int, required=True)
    train.add_argument("--output-dir", required=True)
    train.add_argument("--device", default="auto")
    train.add_argument("--timesteps", type=int, default=600_000)

    select = commands.add_parser("select-sac")
    select.add_argument("--manifest", action="append", required=True)
    select.add_argument("--output", required=True)
    select.add_argument("--device", default="cpu")
    select.add_argument("--episodes-per-seed", type=int, default=20)

    pac = commands.add_parser("fit-pac")
    pac.add_argument("--selection-manifest", required=True)
    pac.add_argument("--stage", type=int, choices=(1, 2, 3), required=True)
    pac.add_argument("--output-dir", required=True)
    pac.add_argument("--device", default="cpu")
    pac.add_argument("--search-samples", type=int, default=50_000)
    pac.add_argument("--selection-samples", type=int, default=20_000)

    diagnose = commands.add_parser("diagnose-pac")
    diagnose.add_argument("--selection-manifest", required=True)
    diagnose.add_argument("--pac-manifest", action="append", required=True)
    diagnose.add_argument("--output-dir", required=True)
    diagnose.add_argument("--device", default="cpu")
    diagnose.add_argument("--max-degree", type=int, default=6)
    diagnose.add_argument("--initial-rollouts", type=int, default=50)
    diagnose.add_argument("--uniform-rollouts", type=int, default=100)

    diagnose_qsbc = commands.add_parser("diagnose-qsbc-lp")
    diagnose_qsbc.add_argument("--controller", required=True)
    diagnose_qsbc.add_argument("--output-dir", required=True)
    diagnose_qsbc.add_argument(
        "--degree",
        type=int,
        action="append",
        dest="degrees",
    )
    diagnose_qsbc.add_argument(
        "--epsbar",
        type=float,
        action="append",
        dest="epsbar_values",
    )
    diagnose_qsbc.add_argument("--grid-x-count", type=int, default=101)
    diagnose_qsbc.add_argument("--grid-y-count", type=int, default=81)
    diagnose_qsbc.add_argument("--time-limit-s", type=float, default=300.0)

    build_c2 = commands.add_parser("build-c2-controller")
    build_c2.add_argument("--diagnostic-report", required=True)
    build_c2.add_argument("--output-dir", required=True)

    controller = commands.add_parser("validate-controller")
    controller.add_argument("--candidate", required=True)
    controller.add_argument("--output-dir", required=True)
    controller.add_argument("--skip-rollouts", action="store_true")

    replay_controller = commands.add_parser("replay-controller")
    replay_controller.add_argument("--validation-report", required=True)
    replay_controller.add_argument("--controller", required=True)
    replay_controller.add_argument("--output-dir", required=True)

    solve = commands.add_parser("solve-qsbc")
    solve.add_argument("--controller", required=True)
    solve.add_argument("--output-dir", required=True)
    solve.add_argument("--time-limit-s", type=float, default=600.0)
    solve.add_argument("--threads", type=int, default=8)

    solve_degree = commands.add_parser("solve-qsbc-degree")
    solve_degree.add_argument("--controller", required=True)
    solve_degree.add_argument("--degree", type=int, choices=(4, 6), required=True)
    solve_degree.add_argument("--output-dir", required=True)
    solve_degree.add_argument("--time-limit-s", type=float, default=600.0)
    solve_degree.add_argument("--threads", type=int, default=8)

    solve_bernstein = commands.add_parser("solve-qsbc-bernstein")
    solve_bernstein.add_argument("--controller", required=True)
    solve_bernstein.add_argument("--output-dir", required=True)
    solve_bernstein.add_argument("--time-limit-s", type=float, default=600.0)

    solve_multiplier = commands.add_parser(
        "solve-qsbc-bernstein-multiplier"
    )
    solve_multiplier.add_argument("--controller", required=True)
    solve_multiplier.add_argument("--output-dir", required=True)
    solve_multiplier.add_argument(
        "--time-limit-s",
        type=float,
        default=900.0,
    )

    solve_g10 = commands.add_parser("solve-qsbc-g10")
    solve_g10.add_argument("--controller", required=True)
    solve_g10.add_argument("--output-dir", required=True)

    solve_g10r = commands.add_parser("solve-qsbc-g10r")
    solve_g10r.add_argument("--controller", required=True)
    solve_g10r.add_argument("--output-dir", required=True)

    solve_g10e = commands.add_parser("solve-qsbc-g10e")
    solve_g10e.add_argument("--controller", required=True)
    solve_g10e.add_argument("--output-dir", required=True)

    solve_g10a = commands.add_parser("solve-qsbc-g10a")
    solve_g10a.add_argument("--controller", required=True)
    solve_g10a.add_argument("--g9-seed", required=True)
    solve_g10a.add_argument("--output-dir", required=True)

    solve_g10a2 = commands.add_parser("solve-qsbc-g10a2")
    solve_g10a2.add_argument("--controller", required=True)
    solve_g10a2.add_argument("--g9-seed", required=True)
    solve_g10a2.add_argument("--output-dir", required=True)

    solve_g10a3 = commands.add_parser("solve-qsbc-g10a3")
    solve_g10a3.add_argument("--controller", required=True)
    solve_g10a3.add_argument("--g9-seed", required=True)
    solve_g10a3.add_argument("--output-dir", required=True)

    solve_g11p49 = commands.add_parser("solve-qsbc-g11p49")
    solve_g11p49.add_argument("--controller", required=True)
    solve_g11p49.add_argument("--output-dir", required=True)

    replay_certificate = commands.add_parser("replay-certificate")
    replay_certificate.add_argument("--certificate", required=True)
    replay_certificate.add_argument("--controller", required=True)
    replay_certificate.add_argument("--output-dir", required=True)

    replay_certificate_v3 = commands.add_parser("replay-certificate-v3")
    replay_certificate_v3.add_argument("--certificate", required=True)
    replay_certificate_v3.add_argument("--controller", required=True)
    replay_certificate_v3.add_argument("--output-dir", required=True)

    replay_certificate_v3p = commands.add_parser("replay-certificate-v3p")
    replay_certificate_v3p.add_argument("--certificate", required=True)
    replay_certificate_v3p.add_argument("--roof-witness", required=True)
    replay_certificate_v3p.add_argument("--controller", required=True)
    replay_certificate_v3p.add_argument("--output-dir", required=True)

    live_exact = commands.add_parser("simulate-exact-certificate-live")
    live_exact.add_argument("--controller", required=True)
    live_exact.add_argument("--certificate", required=True)
    live_exact.add_argument("--output-dir", required=True)
    live_exact.add_argument("--seed", type=int, default=1009)
    live_exact.add_argument(
        "--no-wall-clock",
        action="store_true",
    )
    live_exact.add_argument("--no-console", action="store_true")
    live_exact.add_argument("--no-gif", action="store_true")
    live_exact.add_argument("--display-stride", type=int, default=4)

    dynamic_road = commands.add_parser("simulate-dynamic-road-cbf-live")
    dynamic_road.add_argument("--config", required=True)
    dynamic_road.add_argument("--output-dir", required=True)
    dynamic_road.add_argument("--seed", type=int)
    dynamic_road.add_argument("--no-wall-clock", action="store_true")
    dynamic_road.add_argument("--emit-console", action="store_true")
    dynamic_road.add_argument("--no-gif", action="store_true")
    dynamic_road.add_argument("--display-stride", type=int)
    dynamic_road.add_argument("--expected-trace-sha256")

    dynamic_batch = commands.add_parser(
        "simulate-dynamic-road-cbf-batch"
    )
    dynamic_batch.add_argument("--batch-config", required=True)
    dynamic_batch.add_argument(
        "--catalog",
        choices=("discovery", "validation"),
        required=True,
    )
    dynamic_batch.add_argument("--output-dir", required=True)
    dynamic_batch.add_argument("--expected-metrics-sha256")

    differential_road = commands.add_parser(
        "simulate-differential-road-cbf-live"
    )
    differential_road.add_argument("--config", required=True)
    differential_road.add_argument("--output-dir", required=True)
    differential_road.add_argument("--seed", type=int)
    differential_road.add_argument("--no-wall-clock", action="store_true")
    differential_road.add_argument("--emit-console", action="store_true")
    differential_road.add_argument("--no-gif", action="store_true")
    differential_road.add_argument("--display-stride", type=int)
    differential_road.add_argument("--expected-trace-sha256")

    differential_batch = commands.add_parser(
        "simulate-differential-road-cbf-batch"
    )
    differential_batch.add_argument("--batch-config", required=True)
    differential_batch.add_argument(
        "--catalog",
        choices=("discovery", "validation"),
        required=True,
    )
    differential_batch.add_argument("--output-dir", required=True)
    differential_batch.add_argument("--expected-metrics-sha256")

    gazebo_road = commands.add_parser(
        "simulate-gazebo-classic-road-cbf-live"
    )
    gazebo_road.add_argument("--config", required=True)
    gazebo_road.add_argument("--bridge-binary", required=True)
    gazebo_road.add_argument("--output-dir", required=True)
    gazebo_road.add_argument("--seed", type=int)
    gazebo_road.add_argument("--emit-console", action="store_true")
    gazebo_road.add_argument("--no-gif", action="store_true")
    gazebo_road.add_argument("--display-stride", type=int)
    gazebo_road.add_argument("--gzserver-binary", default="gzserver")
    gazebo_road.add_argument(
        "--live-display",
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    gazebo_road.add_argument(
        "--live-display-host",
        default=None,
    )
    gazebo_road.add_argument("--live-display-port", type=int, default=None)
    gazebo_road.add_argument(
        "--wall-clock",
        action=argparse.BooleanOptionalAction,
        default=None,
    )

    gazebo_batch = commands.add_parser(
        "simulate-gazebo-classic-road-cbf-batch"
    )
    gazebo_batch.add_argument("--batch-config", required=True)
    gazebo_batch.add_argument(
        "--catalog",
        choices=("discovery", "validation"),
        required=True,
    )
    gazebo_batch.add_argument("--bridge-binary", required=True)
    gazebo_batch.add_argument("--output-dir", required=True)
    gazebo_batch.add_argument("--gzserver-binary", default="gzserver")

    gazebo_replay = commands.add_parser(
        "validate-gazebo-classic-road-replay"
    )
    gazebo_replay.add_argument("--config", required=True)
    gazebo_replay.add_argument(
        "--run-manifest",
        action="append",
        required=True,
    )
    gazebo_replay.add_argument("--output-dir", required=True)

    gazebo_render = commands.add_parser(
        "render-gazebo-classic-road-cbf-run"
    )
    gazebo_render.add_argument("--config", required=True)
    gazebo_render.add_argument("--run-manifest", required=True)
    gazebo_render.add_argument("--output-dir", required=True)
    gazebo_render.add_argument("--display-stride", type=int)

    audit = commands.add_parser("audit-artifact")
    audit.add_argument(
        "--kind",
        choices=("controller", "certificate", "selection", "manifest"),
        required=True,
    )
    audit.add_argument("--path", required=True)
    audit.add_argument("--controller")
    return parser


def _print_result(path: Path, **extra: object) -> None:
    print(
        json.dumps(
            {"artifact": str(path.resolve()), **extra},
            sort_keys=True,
            ensure_ascii=False,
        )
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "train-sac":
        from cbf_rover.adapters.training.sac_reference import (
            SACTrainingContract,
            train_sac_reference,
        )

        path = train_sac_reference(
            seed=args.seed,
            output_dir=args.output_dir,
            device=args.device,
            contract=SACTrainingContract(
                total_timesteps=args.timesteps,
                contract_id=(
                    "ST1_SAC_600K_SYMMETRIC_GLOBAL_V1"
                    if args.timesteps == 600_000
                    else f"SMOKE_OVERRIDE_{args.timesteps}_STEPS"
                ),
            ),
        )
        _print_result(path, status="trained")
        return 0

    if args.command == "select-sac":
        from cbf_rover.adapters.training.sac_reference import (
            select_sac_checkpoint,
        )

        path = select_sac_checkpoint(
            args.manifest,
            output_path=args.output,
            device=args.device,
            episodes_per_seed=args.episodes_per_seed,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "selected" else 2

    if args.command == "fit-pac":
        from cbf_rover.methods.polynomial_sbc.pac import fit_pac_stage

        path = fit_pac_stage(
            selection_manifest=args.selection_manifest,
            stage_index=args.stage,
            output_dir=args.output_dir,
            device=args.device,
            search_sample_count=args.search_samples,
            selection_sample_count=args.selection_samples,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "candidate" else 2

    if args.command == "diagnose-pac":
        from cbf_rover.methods.polynomial_sbc.diagnostics import (
            diagnose_pac_failure,
        )

        if args.max_degree < 1:
            raise ValueError("--max-degree must be positive")
        path = diagnose_pac_failure(
            selection_manifest=args.selection_manifest,
            pac_manifests=args.pac_manifest,
            output_dir=args.output_dir,
            device=args.device,
            dense_degrees=tuple(range(1, args.max_degree + 1)),
            initial_rollouts=args.initial_rollouts,
            uniform_rollouts=args.uniform_rollouts,
        )
        _print_result(path, status="diagnosed")
        return 0

    if args.command == "diagnose-qsbc-lp":
        from cbf_rover.methods.polynomial_sbc.qsbc_diagnostics import (
            diagnose_qsbc_expressivity,
        )

        path = diagnose_qsbc_expressivity(
            frozen_controller_path=args.controller,
            output_dir=args.output_dir,
            degrees=(
                tuple(args.degrees)
                if args.degrees is not None
                else (4, 6, 8, 10, 12)
            ),
            epsbar_values=(
                tuple(args.epsbar_values)
                if args.epsbar_values is not None
                else (0.0, 1.0e-6)
            ),
            grid_x_count=args.grid_x_count,
            grid_y_count=args.grid_y_count,
            time_limit_s=args.time_limit_s,
        )
        _print_result(path, status="diagnosed")
        return 0

    if args.command == "build-c2-controller":
        from cbf_rover.methods.polynomial_sbc.piecewise_controller import (
            publish_c2_candidate,
        )

        path = publish_c2_candidate(
            diagnostic_report=args.diagnostic_report,
            output_dir=args.output_dir,
        )
        _print_result(path, status="candidate")
        return 0

    if args.command == "validate-controller":
        from cbf_rover.methods.polynomial_sbc.controller_validation import (
            validate_and_freeze_controller,
        )

        path = validate_and_freeze_controller(
            candidate_path=args.candidate,
            output_dir=args.output_dir,
            run_rollouts=not args.skip_rollouts,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "validated" else 2

    if args.command == "replay-controller":
        from cbf_rover.methods.polynomial_sbc.controller_validation import (
            replay_controller_validation,
        )

        path = replay_controller_validation(
            args.validation_report,
            args.controller,
            args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "validated" else 2

    if args.command in {"solve-qsbc", "solve-qsbc-degree"}:
        from cbf_rover.methods.polynomial_sbc.sos import (
            SOSSynthesisOptions,
            solve_qsbc_degree,
            solve_qsbc_schedule,
        )

        options = SOSSynthesisOptions(
            time_limit_s=args.time_limit_s,
            threads=args.threads,
        )
        if args.command == "solve-qsbc":
            path = solve_qsbc_schedule(
                frozen_controller_path=args.controller,
                output_dir=args.output_dir,
                options=options,
            )
        else:
            path = solve_qsbc_degree(
                frozen_controller_path=args.controller,
                barrier_degree=args.degree,
                output_dir=args.output_dir,
                options=options,
            )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-bernstein":
        from cbf_rover.methods.polynomial_sbc.bernstein_lp import (
            synthesize_bernstein_qsbc_schedule,
        )

        path = synthesize_bernstein_qsbc_schedule(
            frozen_controller_path=args.controller,
            output_dir=args.output_dir,
            time_limit_s=args.time_limit_s,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-bernstein-multiplier":
        from cbf_rover.methods.polynomial_sbc.bernstein_multiplier_lp import (
            synthesize_multiplier_bernstein_qsbc,
        )

        path = synthesize_multiplier_bernstein_qsbc(
            frozen_controller_path=args.controller,
            output_dir=args.output_dir,
            time_limit_s=args.time_limit_s,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-g10":
        from cbf_rover.methods.polynomial_sbc.g10_sparse_multiplier_lp import (
            synthesize_g10_qsbc_schedule,
        )

        path = synthesize_g10_qsbc_schedule(
            frozen_controller_path=args.controller,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-g10r":
        from cbf_rover.methods.polynomial_sbc.g10r_reduced_multiplier_lp import (
            synthesize_g10r_qsbc_schedule,
        )

        path = synthesize_g10r_qsbc_schedule(
            frozen_controller_path=args.controller,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-g10e":
        from cbf_rover.methods.polynomial_sbc.g10e_extended_reduced_lp import (
            synthesize_g10e_qsbc_schedule,
        )

        path = synthesize_g10e_qsbc_schedule(
            frozen_controller_path=args.controller,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-g10a":
        from cbf_rover.methods.polynomial_sbc.g10a_active_set_lp import (
            synthesize_g10a_qsbc_schedule,
        )

        path = synthesize_g10a_qsbc_schedule(
            frozen_controller_path=args.controller,
            g9_seed_path=args.g9_seed,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-g10a2":
        from cbf_rover.methods.polynomial_sbc.g10a2_exact_preseparation import (
            synthesize_g10a2_qsbc_schedule,
        )

        path = synthesize_g10a2_qsbc_schedule(
            frozen_controller_path=args.controller,
            g9_seed_path=args.g9_seed,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-g10a3":
        from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
            synthesize_g10a3_qsbc_schedule,
        )

        path = synthesize_g10a3_qsbc_schedule(
            frozen_controller_path=args.controller,
            g9_seed_path=args.g9_seed,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "solve-qsbc-g11p49":
        from cbf_rover.methods.polynomial_sbc.g11p49_piecewise_roof_lp import (
            synthesize_g11p49_qsbc_schedule,
        )

        path = synthesize_g11p49_qsbc_schedule(
            frozen_controller_path=args.controller,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "ready" else 2

    if args.command == "replay-certificate":
        from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
        from cbf_rover.methods.polynomial_sbc.certificate_validation import (
            validate_qsbc_values,
        )
        from cbf_rover.methods.polynomial_sbc.polynomial import (
            load_frozen_controller,
        )

        controller = load_frozen_controller(args.controller)
        certificate = QSBCertificate.load(
            args.certificate,
            controller=controller,
        )
        if certificate.schema_version == 3:
            raise ValueError("schema-3 certificates require replay-certificate-v3p")
        path = validate_qsbc_values(
            barrier=certificate.barrier,
            q=certificate.q,
            epsbar=certificate.epsbar,
            controller=controller,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "validated" else 2

    if args.command == "replay-certificate-v3":
        from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
        from cbf_rover.methods.polynomial_sbc.certificate_validation import (
            validate_qsbc_values_v3,
        )
        from cbf_rover.methods.polynomial_sbc.polynomial import (
            load_frozen_controller,
        )

        controller = load_frozen_controller(args.controller)
        certificate = QSBCertificate.load(
            args.certificate,
            controller=controller,
        )
        if certificate.schema_version == 3:
            raise ValueError("schema-3 certificates require V3P, not legacy V3")
        path = validate_qsbc_values_v3(
            barrier=certificate.barrier,
            q=certificate.q,
            epsbar=certificate.epsbar,
            controller=controller,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "validated" else 2

    if args.command == "replay-certificate-v3p":
        from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
        from cbf_rover.methods.polynomial_sbc.g11p49_piecewise_roof_lp import (
            _v3p_release_gate,
        )
        from cbf_rover.methods.polynomial_sbc.piecewise_barrier import (
            PiecewiseRoofWitness2D,
        )
        from cbf_rover.methods.polynomial_sbc.piecewise_roof_validation import (
            validate_piecewise_qsbc_v3p,
        )
        from cbf_rover.methods.polynomial_sbc.polynomial import (
            load_frozen_controller,
        )

        controller = load_frozen_controller(args.controller)
        certificate = QSBCertificate.load(
            args.certificate,
            controller=controller,
        )
        if certificate.schema_version != 3:
            raise ValueError("V3P replay requires a schema-3 certificate")
        witness = PiecewiseRoofWitness2D.load(args.roof_witness)
        witness_bindings = {
            "roof_witness_content_sha256": witness.sha256,
            "barrier_payload_sha256": witness.barrier_payload_sha256,
            "controller_content_sha256": witness.controller_content_sha256,
            "formal_model_contract_sha256": (
                witness.formal_model_contract_sha256
            ),
            "geometry_catalog_sha256": witness.geometry_catalog_sha256,
            "barrier_degree": witness.barrier_degree,
            "roof_degree": witness.roof_degree,
        }
        for name, actual in witness_bindings.items():
            proof_expected = certificate.proof_witness.get(name)
            validation_expected = certificate.validation.get(name)
            if actual != proof_expected or actual != validation_expected:
                raise ValueError(f"V3P external witness {name} mismatch")
        path = validate_piecewise_qsbc_v3p(
            barrier=certificate.barrier,
            roof_witness=witness,
            q=certificate.q,
            epsbar=certificate.epsbar,
            controller=controller,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        release_gate_passed, release_gate_failures = _v3p_release_gate(
            validation=payload,
            validation_directory=path.parent,
            barrier=certificate.barrier,
            witness=witness,
            controller=controller,
        )
        if not release_gate_passed:
            raise ValueError(
                "V3P replay release gate failed: "
                + ", ".join(release_gate_failures)
            )
        if payload.get("report_content_sha256") != certificate.validation.get(
            "validation_report_content_sha256"
        ):
            raise ValueError("V3P replay report content hash mismatch")
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "validated" else 2

    if args.command == "simulate-exact-certificate-live":
        from cbf_rover.adapters.exact.live_certificate import (
            simulate_exact_certificate_live,
        )

        path = simulate_exact_certificate_live(
            controller_path=args.controller,
            certificate_path=args.certificate,
            output_dir=args.output_dir,
            seed=args.seed,
            wall_clock=not args.no_wall_clock,
            emit_console=not args.no_console,
            render_gif=not args.no_gif,
            display_stride=args.display_stride,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "goal_reached" else 2

    if args.command == "simulate-dynamic-road-cbf-live":
        from cbf_rover.adapters.exact.dynamic_road import (
            simulate_dynamic_road_cbf_live,
        )

        path = simulate_dynamic_road_cbf_live(
            config_path=args.config,
            output_dir=args.output_dir,
            seed=args.seed,
            wall_clock=not args.no_wall_clock,
            emit_console=args.emit_console,
            render_gif=not args.no_gif,
            display_stride=args.display_stride,
            expected_trace_sha256=args.expected_trace_sha256,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return (
            0
            if payload["status"] in {"accepted", "provisional_success"}
            else 2
        )

    if args.command == "simulate-dynamic-road-cbf-batch":
        from cbf_rover.adapters.exact.dynamic_road_batch import (
            run_dynamic_road_batch,
        )

        path = run_dynamic_road_batch(
            batch_config_path=args.batch_config,
            catalog_name=args.catalog,
            output_dir=args.output_dir,
            expected_metrics_sha256=args.expected_metrics_sha256,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return (
            0
            if payload["status"]
            in {
                "accepted",
                "provisional_success",
                "diagnostic_complete",
            }
            else 2
        )

    if args.command == "simulate-differential-road-cbf-live":
        from cbf_rover.adapters.exact.differential_road import (
            simulate_differential_road_cbf,
        )
        from cbf_rover.adapters.exact.differential_road_contract import (
            load_differential_road_contract,
        )

        contract = load_differential_road_contract(args.config)
        path = simulate_differential_road_cbf(
            contract=contract,
            output_dir=args.output_dir,
            seed=args.seed,
            wall_clock=not args.no_wall_clock,
            emit_console=args.emit_console,
            render_gif=not args.no_gif,
            display_stride=args.display_stride,
            expected_trace_sha256=args.expected_trace_sha256,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return (
            0
            if payload["status"] in {"accepted", "provisional_success"}
            else 2
        )

    if args.command == "simulate-differential-road-cbf-batch":
        from cbf_rover.adapters.exact.differential_road_batch import (
            run_differential_road_batch,
        )

        path = run_differential_road_batch(
            batch_config_path=args.batch_config,
            catalog_name=args.catalog,
            output_dir=args.output_dir,
            expected_metrics_sha256=args.expected_metrics_sha256,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return (
            0
            if payload["status"]
            in {
                "accepted",
                "provisional_success",
                "diagnostic_complete",
            }
            else 2
        )

    if args.command == "simulate-gazebo-classic-road-cbf-live":
        from cbf_rover.adapters.gazebo.differential_road import (
            simulate_gazebo_differential_road_cbf,
        )
        from cbf_rover.adapters.gazebo.differential_road_contract import (
            load_gazebo_differential_road_contract,
        )

        contract = load_gazebo_differential_road_contract(args.config)
        path = simulate_gazebo_differential_road_cbf(
            contract=contract,
            bridge_binary=args.bridge_binary,
            output_dir=args.output_dir,
            seed=args.seed,
            emit_console=args.emit_console,
            render_gif=not args.no_gif,
            display_stride=args.display_stride,
            gzserver_binary=args.gzserver_binary,
            live_display=args.live_display,
            live_display_host=args.live_display_host,
            live_display_port=args.live_display_port,
            wall_clock=args.wall_clock,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return (
            0
            if payload["status"] in {"accepted", "provisional_success"}
            else 2
        )

    if args.command == "simulate-gazebo-classic-road-cbf-batch":
        from cbf_rover.adapters.gazebo.multiseed import (
            run_gazebo_road_batch,
        )

        path = run_gazebo_road_batch(
            batch_config_path=args.batch_config,
            catalog_name=args.catalog,
            bridge_binary=args.bridge_binary,
            output_dir=args.output_dir,
            gzserver_binary=args.gzserver_binary,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return (
            0
            if payload["status"]
            in {"provisional_success", "diagnostic_complete"}
            else 2
        )

    if args.command == "validate-gazebo-classic-road-replay":
        from cbf_rover.adapters.gazebo.replay_validation import (
            load_gazebo_replay_validation_spec,
            validate_gazebo_replay,
        )

        spec = load_gazebo_replay_validation_spec(args.config)
        path = validate_gazebo_replay(
            spec=spec,
            run_manifest_paths=args.run_manifest,
            output_dir=args.output_dir,
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=payload["status"])
        return 0 if payload["status"] == "accepted" else 2

    if args.command == "render-gazebo-classic-road-cbf-run":
        from cbf_rover.adapters.gazebo.differential_road_contract import (
            load_gazebo_differential_road_contract,
        )
        from cbf_rover.adapters.gazebo.run_render import (
            render_gazebo_differential_road_run,
        )

        contract = load_gazebo_differential_road_contract(args.config)
        path = render_gazebo_differential_road_run(
            contract=contract,
            run_manifest_path=args.run_manifest,
            output_dir=args.output_dir,
            display_stride=args.display_stride,
        )
        _print_result(path, status="rendered")
        return 0

    path = Path(args.path).resolve()
    if args.kind == "controller":
        from cbf_rover.methods.polynomial_sbc.polynomial import (
            load_frozen_controller,
        )

        artifact = load_frozen_controller(path)
        _print_result(path, status="valid", content_sha256=artifact.sha256)
    elif args.kind == "certificate":
        from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
        from cbf_rover.methods.polynomial_sbc.polynomial import (
            load_frozen_controller,
        )

        controller = (
            load_frozen_controller(args.controller)
            if args.controller
            else None
        )
        artifact = QSBCertificate.load(path, controller=controller)
        _print_result(
            path,
            status="structurally_valid",
            content_sha256=artifact.sha256,
            formal_status_requires_v3_replay=True,
        )
    else:
        payload = json.loads(path.read_text(encoding="utf-8"))
        _print_result(path, status=str(payload.get("status", "readable")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
