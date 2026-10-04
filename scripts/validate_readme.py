#!/usr/bin/env python3
"""
Validates README.md for:
1. GitHub-compatible LaTeX syntax (no \\operatorname, no unescaped underscores in \\text{}, balanced math delimiters).
2. Synchronization between serial CLI commands in Core/Src/cmd.c and the README command table.
"""

import sys
import re
from pathlib import Path

def validate_latex(readme_path: Path) -> list[str]:
    errors = []
    lines = readme_path.read_text(encoding="utf-8").splitlines()

    dollar_count = 0
    in_code_block = False

    for line_num, line in enumerate(lines, start=1):
        # Track fenced code blocks (```...```)
        if line.strip().startswith("```"):
            in_code_block = not in_code_block
            continue

        if in_code_block:
            continue

        # Check for disallowed \operatorname macro (causes GitHub rendering error)
        if r"\operatorname" in line:
            errors.append(f"Line {line_num}: Disallowed macro '\\operatorname' found. Use '\\mathrm' or plain text instead.")

        # Check for underscores inside \text{...} (causes GitHub error: "'_' allowed only in math mode")
        for match in re.finditer(r"\\text\{([^}]*_[^}]*)\}", line):
            errors.append(f"Line {line_num}: Underscore found inside '\\text{{{match.group(1)}}}'. Avoid underscores in \\text{{}} for GitHub compatibility.")

        # Check for indented $$ delimiters (causes GitHub math rendering to fail inside lists)
        if line.strip().startswith("$$") and (line.startswith(" ") or line.startswith("\t")):
            errors.append(f"Line {line_num}: Indented display math delimiter '$$' found. GitHub requires display math delimiters to be unindented (column 0).")

        # Count $$ delimiters on non-code lines
        dollar_count += line.count("$$")

    if dollar_count % 2 != 0:
        errors.append(f"Unbalanced display math delimiters ($$): found {dollar_count} total occurrences (must be an even count).")

    return errors


def validate_cli_commands(readme_path: Path, cmd_c_path: Path) -> list[str]:
    errors = []
    if not cmd_c_path.exists():
        errors.append(f"Command source file not found: {cmd_c_path}")
        return errors

    cmd_c_content = cmd_c_path.read_text(encoding="utf-8")
    readme_content = readme_path.read_text(encoding="utf-8")

    # Extract commands from static const Command_t commands[] in cmd.c
    match = re.search(r"static\s+const\s+Command_t\s+commands\[\]\s*=\s*\{([\s\S]*?)\n\};", cmd_c_content)
    if not match:
        errors.append("Could not locate 'commands[]' array in Core/Src/cmd.c")
        return errors

    commands_block = match.group(1)
    registered_cmds = re.findall(r'\{\s*"([^"]+)"', commands_block)

    if not registered_cmds:
        errors.append("No commands extracted from commands[] array in Core/Src/cmd.c")
        return errors

    # Check that each command appears as an entry in the README command table: | `cmd` |
    for cmd in registered_cmds:
        pattern = rf"\|\s*`{re.escape(cmd)}`\s*\|"
        if not re.search(pattern, readme_content):
            errors.append(f"CLI command '{cmd}' from cmd.c is missing from the command reference table in README.md")

    return errors


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    readme_path = repo_root / "README.md"
    cmd_c_path = repo_root / "Core" / "Src" / "cmd.c"

    if not readme_path.exists():
        print(f"[ERROR] README.md not found at {readme_path}", file=sys.stderr)
        return 1

    print(f"Validating {readme_path.name}...")
    errors = []
    errors.extend(validate_latex(readme_path))
    errors.extend(validate_cli_commands(readme_path, cmd_c_path))

    if errors:
        print(f"\n[FAIL] Found {len(errors)} issue(s) in {readme_path.name}:", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        return 1

    print("[PASS] README.md validation successful (LaTeX syntax and CLI command synchronization verified).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
