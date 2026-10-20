#!/usr/bin/env python3
"""
scripts/release.py

Automates release preparation, validation, git tagging, and publishing for StepperDriverEmulator.

Usage:
    python scripts/release.py <version>             # e.g., v1.0.0 or 1.0.0
    python scripts/release.py --bump patch          # bumps patch (v1.0.0 -> v1.0.1)
    python scripts/release.py --bump minor          # bumps minor (v1.0.0 -> v1.1.0)
    python scripts/release.py --bump major          # bumps major (v1.0.0 -> v2.0.0)
    python scripts/release.py [options] --dry-run   # runs all validations without tagging or pushing
    python scripts/release.py [options] --yes       # skips interactive confirmation prompts
"""

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path


def run_cmd(cmd: list[str], cwd: Path | None = None, check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            cmd,
            cwd=cwd,
            check=check,
            capture_output=capture,
            text=True,
            encoding="utf-8"
        )
    except subprocess.CalledProcessError as e:
        if capture:
            print(f"[ERROR] Command failed: {' '.join(cmd)}\n{e.stderr}", file=sys.stderr)
        raise


def get_current_branch(repo_root: Path) -> str:
    res = run_cmd(["git", "branch", "--show-current"], cwd=repo_root, capture=True)
    return res.stdout.strip()


def is_working_tree_clean(repo_root: Path) -> bool:
    res = run_cmd(["git", "status", "--porcelain"], cwd=repo_root, capture=True)
    return len(res.stdout.strip()) == 0


def get_latest_tag(repo_root: Path) -> str | None:
    res = run_cmd(["git", "tag", "--list", "v*"], cwd=repo_root, capture=True)
    tags = [t.strip() for t in res.stdout.splitlines() if t.strip()]
    if not tags:
        return None
    
    # Sort semantic versions
    def parse_semver(tag: str):
        match = re.match(r"^v?(\d+)\.(\d+)\.(\d+)", tag)
        if match:
            return tuple(int(x) for x in match.groups())
        return (0, 0, 0)

    tags.sort(key=parse_semver)
    return tags[-1]


def compute_bumped_version(latest_tag: str | None, bump_type: str) -> str:
    if latest_tag is None:
        base = (0, 0, 0)
    else:
        match = re.match(r"^v?(\d+)\.(\d+)\.(\d+)", latest_tag)
        if not match:
            base = (0, 0, 0)
        else:
            base = tuple(int(x) for x in match.groups())

    major, minor, patch = base
    if bump_type == "patch":
        patch += 1
    elif bump_type == "minor":
        minor += 1
        patch = 0
    elif bump_type == "major":
        major += 1
        minor = 0
        patch = 0
    else:
        raise ValueError(f"Unknown bump type: {bump_type}")

    return f"v{major}.{minor}.{patch}"


def normalize_version(version_str: str) -> str:
    v = version_str.strip()
    if not v.startswith("v"):
        v = "v" + v
    if not re.match(r"^v\d+\.\d+\.\d+(-[0-9A-Za-z.-]+)?$", v):
        raise ValueError(f"Invalid semantic version: '{version_str}'. Expected format: vX.Y.Z or X.Y.Z")
    return v


def tag_exists(repo_root: Path, tag: str) -> bool:
    res = run_cmd(["git", "tag", "--list", tag], cwd=repo_root, capture=True)
    return len(res.stdout.strip()) > 0


def tag_exists_on_remote(repo_root: Path, tag: str) -> bool:
    try:
        res = run_cmd(["git", "ls-remote", "--tags", "origin", f"refs/tags/{tag}"], cwd=repo_root, capture=True)
        return len(res.stdout.strip()) > 0
    except subprocess.CalledProcessError:
        return False


def run_validations(repo_root: Path) -> None:
    print("\n--- Running Presubmit Validations ---")

    # 1. README & LaTeX validation
    print("[1/5] Validating documentation (validate_readme.py)...")
    run_cmd([sys.executable, str(repo_root / "scripts" / "validate_readme.py")], cwd=repo_root)

    # 2. Host unit tests
    print("[2/5] Building and executing host unit tests...")
    host_test_dir = repo_root / "build" / "host-test"
    if host_test_dir.exists():
        run_cmd(["cmake", "--build", "build/host-test", "--config", "Debug"], cwd=repo_root)
        run_cmd(["ctest", "--test-dir", "build/host-test", "-C", "Debug", "--output-on-failure"], cwd=repo_root)
    else:
        # Fall back to host-test-ninja if present
        run_cmd(["ctest", "--preset", "host-test-ninja", "--output-on-failure"], cwd=repo_root)

    # 3. Release firmware build
    print("[3/5] Building ARM Cortex-M0 Release firmware...")
    run_cmd(["cmake", "--build", "build/Release", "--config", "Release"], cwd=repo_root)

    # 4. Sizing and disallowed symbols checks
    print("[4/5] Checking firmware size and disallowed symbols...")
    elf_path = repo_root / "build" / "Release" / "StepperDriverEmulator.elf"
    if not elf_path.exists():
        raise FileNotFoundError(f"Release ELF not found at {elf_path}")
    bin_path = repo_root / "build" / "Release" / "StepperDriverEmulator.bin"
    if not bin_path.exists():
        raise FileNotFoundError(f"Release binary not found at {bin_path}")
    hex_path = repo_root / "build" / "Release" / "StepperDriverEmulator.hex"
    if not hex_path.exists():
        raise FileNotFoundError(f"Release HEX not found at {hex_path}")
    run_cmd([sys.executable, str(repo_root / "scripts" / "check_firmware_size.py"), str(elf_path)], cwd=repo_root)
    run_cmd([sys.executable, str(repo_root / "scripts" / "check_disallowed_symbols.py"), str(elf_path)], cwd=repo_root)

    # 5. Go Flasher build
    print("[5/5] Verifying Go flasher builds cleanly...")
    run_cmd(["go", "build", "./cmd/flasher"], cwd=repo_root)
    # Clean up generated executable if created in root
    for candidate in ("flasher.exe", "flasher"):
        exe_file = repo_root / candidate
        if exe_file.exists():
            exe_file.unlink()

    print("[PASS] All presubmit validations succeeded.\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Create and publish a validated release.")
    parser.add_argument("version", nargs="?", help="Version tag to release (e.g. v1.0.0 or 1.0.0)")
    parser.add_argument("--bump", choices=["patch", "minor", "major"], help="Bump version type relative to latest tag")
    parser.add_argument("--dry-run", action="store_true", help="Run all validations without tagging or pushing")
    parser.add_argument("--yes", "-y", action="store_true", help="Skip confirmation prompts")
    parser.add_argument("--allow-branch", default="main", help="Allowed branch for release (default: main)")

    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parent.parent

    # Check git presence
    try:
        run_cmd(["git", "rev-parse", "--is-inside-work-tree"], cwd=repo_root, capture=True)
    except Exception:
        print("[ERROR] Not inside a git repository.", file=sys.stderr)
        return 1

    # Branch check
    current_branch = get_current_branch(repo_root)
    if current_branch != args.allow_branch:
        print(f"[ERROR] Releases must be created from branch '{args.allow_branch}'. Current branch is '{current_branch}'.", file=sys.stderr)
        return 1

    # Working tree clean check
    if not is_working_tree_clean(repo_root) and not args.dry_run:
        print("[ERROR] Working tree has uncommitted changes. Please commit or stash them before releasing.", file=sys.stderr)
        return 1

    # Determine target version
    latest_tag = get_latest_tag(repo_root)
    if args.bump:
        target_version = compute_bumped_version(latest_tag, args.bump)
    elif args.version:
        try:
            target_version = normalize_version(args.version)
        except ValueError as e:
            print(f"[ERROR] {e}", file=sys.stderr)
            return 1
    else:
        print(f"Latest tag: {latest_tag or '(none)'}")
        suggested = compute_bumped_version(latest_tag, "patch")
        if args.yes:
            target_version = suggested
        else:
            try:
                user_val = input(f"Enter release version [{suggested}]: ").strip()
                target_version = normalize_version(user_val if user_val else suggested)
            except (KeyboardInterrupt, EOFError):
                print("\nAborted.")
                return 1

    print(f"Target release version: {target_version} (Latest: {latest_tag or 'none'})")

    # Check if tag already exists
    if tag_exists(repo_root, target_version):
        print(f"[ERROR] Git tag '{target_version}' already exists locally.", file=sys.stderr)
        return 1
    if tag_exists_on_remote(repo_root, target_version):
        print(f"[ERROR] Git tag '{target_version}' already exists on remote 'origin'.", file=sys.stderr)
        return 1

    # Run presubmit validations
    try:
        run_validations(repo_root)
    except Exception as e:
        print(f"[FAIL] Presubmit validation failed: {e}", file=sys.stderr)
        return 1

    head_commit = run_cmd(["git", "log", "-1", "--oneline"], cwd=repo_root, capture=True).stdout.strip()
    print(f"Ready to tag release {target_version} at commit:\n  {head_commit}\n")

    if args.dry_run:
        print(f"[DRY-RUN] Validation completed successfully. Would tag {target_version} and push to origin.")
        return 0

    if not args.yes:
        try:
            confirm = input(f"Create annotated tag '{target_version}' and push to origin? [y/N]: ").strip().lower()
            if confirm not in ("y", "yes"):
                print("Aborted. Tag was not created.")
                return 0
        except (KeyboardInterrupt, EOFError):
            print("\nAborted.")
            return 0

    # Create annotated tag
    print(f"Creating annotated tag: {target_version}...")
    run_cmd(["git", "tag", "-a", target_version, "-m", f"Release {target_version}"], cwd=repo_root)

    # Push tag to origin
    print(f"Pushing tag {target_version} to origin...")
    try:
        run_cmd(["git", "push", "origin", target_version], cwd=repo_root)
    except subprocess.CalledProcessError:
        print(f"[WARNING] Failed to push tag '{target_version}' to remote 'origin'.", file=sys.stderr)
        print("You can manually push it using: git push origin " + target_version, file=sys.stderr)
        return 1

    print(f"\n[SUCCESS] Tag '{target_version}' pushed to origin successfully!")
    print("GitHub Actions will now build and publish the release with binary assets.")
    print("Monitor progress under the 'Actions' tab on GitHub.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

