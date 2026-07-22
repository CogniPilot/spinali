# Copyright (c) 2026 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

'''release_command.py

West extension command for releasing firmware images to the firmware_releases repository.
This command:
1. Reads the built firmware binary from the build directory
2. Extracts version info from the Zephyr build
3. Creates/updates the manifest (latest.json)
4. Commits and pushes to the firmware_releases repository
'''

import os
import sys
import json
import hashlib
import subprocess
import shutil
from pathlib import Path
from datetime import datetime, timezone

from west.commands import WestCommand
from west import log


class ReleaseCommand(WestCommand):

    def __init__(self):
        super().__init__(
            'release_image',
            'Release firmware image to firmware_releases repository',
            '''\
Releases a built firmware image to the CogniPilot firmware_releases repository.

This command:
1. Reads the firmware binary from the build directory
2. Computes SHA256 hash and file size
3. Extracts version info from VERSION file or .config
4. Creates/updates the firmware manifest (latest.json)
5. Copies the binary with versioned filename
6. Optionally commits and pushes to the repository

Example usage:
  west release_image -d build/mr_mcxn_t1/optical_flow
  west release_image -d build/spinali/cerebri --push
''')

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description
        )

        parser.add_argument(
            '-d', '--build-dir',
            required=True,
            help='Path to the Zephyr build directory (e.g., build/mr_mcxn_t1/optical_flow)'
        )
        parser.add_argument(
            '-r', '--repo-path',
            default=None,
            help='Path to firmware_releases repository (default: auto-detect from FIRMWARE_RELEASES_PATH env or ../firmware_releases)'
        )
        parser.add_argument(
            '-b', '--board',
            default=None,
            help='Board name (default: extracted from build directory path)'
        )
        parser.add_argument(
            '-a', '--app',
            default=None,
            help='Application name (default: extracted from build directory path)'
        )
        parser.add_argument(
            '--push',
            action='store_true',
            help='Commit and push changes to remote repository'
        )
        parser.add_argument(
            '--changelog',
            default=None,
            help='Changelog message for this release'
        )
        parser.add_argument(
            '--dry-run',
            action='store_true',
            help='Show what would be done without making changes'
        )

        return parser

    def do_run(self, args, unknown_args):
        build_dir = Path(args.build_dir).resolve()

        # Validate build directory
        if not build_dir.exists():
            log.die(f'Build directory does not exist: {build_dir}')

        # Find the firmware binary
        # Sysbuild: resolve the default domain dir via domains.yaml
        domains_file = build_dir / 'domains.yaml'
        if domains_file.exists():
            import yaml
            with open(domains_file) as f:
                domains = yaml.safe_load(f)
            build_dir = build_dir / domains['default']

        # MCUboot signed binary is preferred, otherwise use zephyr.bin
        zephyr_dir = build_dir / 'zephyr'
        if not zephyr_dir.exists():
            log.die(f'Not a valid Zephyr build directory (no zephyr/ subdir): {build_dir}')

        # Look for MCUboot signed binary first, then fallback to zephyr.bin
        binary_path = None
        for name in ['zephyr.signed.bin', 'zephyr.bin']:
            candidate = zephyr_dir / name
            if candidate.exists():
                binary_path = candidate
                break

        if binary_path is None:
            log.die(f'No firmware binary found in {zephyr_dir}')

        log.inf(f'Found firmware binary: {binary_path}')

        # Extract board and app from build path if not specified
        # Expected pattern: build/{board}/{app}
        board = args.board
        app = args.app

        if board is None or app is None:
            # Try to extract from path
            parts = build_dir.parts
            if len(parts) >= 2:
                if board is None:
                    board = parts[-2]
                if app is None:
                    app = parts[-1]

        if board is None or app is None:
            log.die('Could not determine board/app from path. Please specify with --board and --app')

        log.inf(f'Board: {board}, App: {app}')

        # Extract version information
        version = self._extract_version(build_dir)
        if version is None:
            log.die('Could not extract version information from build')

        log.inf(f'Version: {version}')

        # Read binary and compute MCUboot hash
        binary_data = binary_path.read_bytes()
        file_size = len(binary_data)

        log.inf(f'Binary size: {file_size} bytes')

        # Compute MCUboot image hash (what MCUmgr will report)
        # This is the only hash we use - for both download and post-update verification
        mcuboot_hash = self._compute_mcuboot_hash(binary_path)
        if mcuboot_hash:
            log.inf(f'MCUboot hash: {mcuboot_hash[:16]}...')
        else:
            log.die('Could not compute MCUboot hash - binary must be a valid MCUboot image')

        # Find firmware_releases repository
        repo_path = self._find_repo(args.repo_path)
        if repo_path is None:
            log.die('Could not find firmware_releases repository. Set FIRMWARE_RELEASES_PATH or use --repo-path')

        log.inf(f'Repository: {repo_path}')

        if args.dry_run:
            log.inf('=== DRY RUN - No changes will be made ===')

        # Create directory structure: {repo}/{board}/{app}/
        target_dir = repo_path / board / app
        if not args.dry_run:
            target_dir.mkdir(parents=True, exist_ok=True)

        # Versioned binary filename
        binary_filename = f'{version}.bin'
        target_binary = target_dir / binary_filename

        # Construct download URL (GitHub Pages pattern)
        download_url = f'https://firmware.cognipilot.org/{board}/{app}/{binary_filename}'

        # Create release entry
        release = {
            'version': version,
            'date': datetime.now(timezone.utc).isoformat(),
            'mcuboot_hash': mcuboot_hash,
            'size': file_size,
            'url': download_url,
        }

        if args.changelog:
            release['changelog'] = args.changelog

        # Load or create manifest
        manifest_path = target_dir / 'latest.json'
        manifest = self._load_manifest(manifest_path, board, app)

        # Update manifest
        old_latest = manifest.get('latest')
        if old_latest and old_latest.get('version') != version:
            # Move current latest to previous
            if 'previous' not in manifest:
                manifest['previous'] = []
            manifest['previous'].insert(0, old_latest)
            # Keep only last 5 previous versions
            manifest['previous'] = manifest['previous'][:5]

        manifest['latest'] = release

        log.inf(f'Target: {target_binary}')
        log.inf(f'Manifest: {manifest_path}')

        if args.dry_run:
            log.inf('Would write manifest:')
            log.inf(json.dumps(manifest, indent=2))
            return

        # Write binary
        shutil.copy2(binary_path, target_binary)
        log.inf(f'Copied binary to {target_binary}')

        # Write manifest
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        log.inf(f'Wrote manifest to {manifest_path}')

        # Optionally commit and push
        if args.push:
            self._git_commit_and_push(repo_path, board, app, version)
        else:
            log.inf('Changes written locally. Use --push to commit and push to remote.')

    def _extract_version(self, build_dir: Path) -> str | None:
        """Extract version from build directory.

        Tries multiple sources:
        1. VERSION file in build directory
        2. .config for CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION
        3. autoconf.h for CONFIG_APP_VERSION
        """

        # Try VERSION file first (Zephyr application VERSION)
        version_file = build_dir / 'zephyr' / 'VERSION'
        if version_file.exists():
            try:
                content = version_file.read_text().strip()
                # VERSION file typically contains: MAJOR.MINOR.PATCHLEVEL
                if content:
                    return content
            except Exception:
                pass

        # Try .config for MCUboot version
        config_file = build_dir / 'zephyr' / '.config'
        if config_file.exists():
            try:
                content = config_file.read_text()
                for line in content.splitlines():
                    if 'CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION=' in line:
                        # Format: CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="1.2.3"
                        version = line.split('=')[1].strip().strip('"')
                        if version:
                            return version
            except Exception:
                pass

        # Try autoconf.h
        autoconf = build_dir / 'zephyr' / 'include' / 'generated' / 'autoconf.h'
        if autoconf.exists():
            try:
                content = autoconf.read_text()
                for line in content.splitlines():
                    if '#define CONFIG_APP_VERSION_STRING' in line:
                        # Format: #define CONFIG_APP_VERSION_STRING "1.2.3"
                        parts = line.split('"')
                        if len(parts) >= 2:
                            return parts[1]
            except Exception:
                pass

        # Fallback: Try to get git describe
        try:
            result = subprocess.run(
                ['git', 'describe', '--tags', '--always'],
                capture_output=True,
                text=True,
                cwd=build_dir.parent
            )
            if result.returncode == 0:
                version = result.stdout.strip()
                if version:
                    return version
        except Exception:
            pass

        return None

    def _compute_mcuboot_hash(self, binary_path: Path) -> str | None:
        """Compute the MCUboot image hash from a signed binary.

        MCUboot computes the image hash as SHA256 over:
        - Image header (first 32 bytes)
        - Protected TLVs (if present)
        - Image payload

        This is what MCUmgr's image_state command returns and is used for
        post-update verification.
        """
        import struct

        try:
            data = binary_path.read_bytes()

            # MCUboot image header is 32 bytes
            # struct image_header {
            #     uint32_t ih_magic;      // IMAGE_MAGIC = 0x96f3b83d
            #     uint32_t ih_load_addr;
            #     uint16_t ih_hdr_size;   // Header size
            #     uint16_t ih_protect_tlv_size;  // Protected TLV area size
            #     uint32_t ih_img_size;   // Image payload size (excludes header, TLVs)
            #     uint32_t ih_flags;
            #     struct image_version ih_ver;  // 8 bytes
            #     uint32_t _pad1;
            # };

            if len(data) < 32:
                log.wrn('Binary too small to be MCUboot image')
                return None

            magic = struct.unpack('<I', data[0:4])[0]
            if magic != 0x96f3b83d:
                log.wrn(f'Not an MCUboot image (magic=0x{magic:08x}, expected 0x96f3b83d)')
                return None

            hdr_size = struct.unpack('<H', data[8:10])[0]
            protect_tlv_size = struct.unpack('<H', data[10:12])[0]
            img_size = struct.unpack('<I', data[12:16])[0]

            log.dbg(f'MCUboot: hdr_size={hdr_size}, protect_tlv_size={protect_tlv_size}, img_size={img_size}')

            # Hash covers: header + protected TLVs + payload
            # (does NOT include the trailing TLV area with signature)
            hash_size = hdr_size + protect_tlv_size + img_size

            if len(data) < hash_size:
                log.wrn(f'Binary truncated: need {hash_size} bytes, have {len(data)}')
                return None

            # Compute SHA256 over the hashable region
            hash_data = data[:hash_size]
            mcuboot_hash = hashlib.sha256(hash_data).hexdigest()

            return mcuboot_hash

        except Exception as e:
            log.wrn(f'Failed to compute MCUboot hash: {e}')
            return None

    def _find_repo(self, explicit_path: str | None) -> Path | None:
        """Find the firmware_releases repository."""

        # Explicit path
        if explicit_path:
            path = Path(explicit_path).resolve()
            if path.exists():
                return path
            return None

        # Environment variable
        env_path = os.environ.get('FIRMWARE_RELEASES_PATH')
        if env_path:
            path = Path(env_path).resolve()
            if path.exists():
                return path

        # Try common locations relative to workspace
        workspace = Path.cwd()
        candidates = [
            workspace / 'firmware_releases',
            workspace.parent / 'firmware_releases',
            Path.home() / 'cognipilot' / 'firmware_releases',
        ]

        for candidate in candidates:
            if candidate.exists():
                return candidate

        return None

    def _load_manifest(self, path: Path, board: str, app: str) -> dict:
        """Load existing manifest or create new one."""
        if path.exists():
            try:
                with open(path) as f:
                    return json.load(f)
            except Exception as e:
                log.wrn(f'Could not load existing manifest: {e}')

        # Create new manifest
        return {
            'board': board,
            'app': app,
            'latest': None,
            'previous': []
        }

    def _git_commit_and_push(self, repo_path: Path, board: str, app: str, version: str):
        """Commit and push changes to remote."""
        try:
            # Add changes
            subprocess.run(
                ['git', 'add', f'{board}/{app}'],
                cwd=repo_path,
                check=True
            )

            # Commit with signed-off-by
            commit_msg = f'Release {board}/{app} v{version}'
            subprocess.run(
                ['git', 'commit', '-s', '-m', commit_msg],
                cwd=repo_path,
                check=True
            )
            log.inf(f'Committed: {commit_msg}')

            # Push
            subprocess.run(
                ['git', 'push'],
                cwd=repo_path,
                check=True
            )
            log.inf('Pushed to remote')

        except subprocess.CalledProcessError as e:
            log.err(f'Git operation failed: {e}')
            log.inf('You may need to manually commit and push the changes.')
