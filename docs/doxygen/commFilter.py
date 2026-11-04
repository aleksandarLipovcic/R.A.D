import sys
import os
import re


def filter_file(filename):
    # Force UTF-8 stdout so non-ASCII characters in comments don't crash
    # the filter when Doxygen captures this script's output via a pipe.
    sys.stdout.reconfigure(encoding='utf-8')

    ext = os.path.splitext(filename)[1].lower()

    try:
        with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
    except OSError as e:
        # Fail loudly on stderr (so it shows in the Doxygen run log)
        # but still exit cleanly instead of leaving Doxygen with no output.
        sys.stderr.write(f"commFilter.py: could not read '{filename}': {e}\n")
        return

    for line in lines:

        if ext in ['.cpp', '.h', '.hpp', '.c', '.cc', '.cxx']:
            # Match // that is NOT already /// and NOT part of a URL (//)
            # Lookbehind (?<!:) prevents matching https://
            # The replacement targets the // directly without consuming
            # surrounding whitespace into a capture group
            line = re.sub(r'(?<!:)//(?!/)', '///', line)

        elif ext == '.py':
            stripped = line.strip()
            # Skip shebangs, blank lines, and already-converted ## comments
            if (
                stripped.startswith('#')
                and not stripped.startswith('#!')
                and not stripped.startswith('##')
            ):
                # Replace only the FIRST # on the line with ##
                # Preserves indentation and inline comment positioning
                line = re.sub(r'#(?!#)', '##', line, count=1)

        sys.stdout.write(line)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        filter_file(sys.argv[1])
    else:
        sys.stderr.write("commFilter.py: no filename provided\n")