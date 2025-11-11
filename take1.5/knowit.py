import os
from pathlib import Path

def compile_project_to_single_file(root_dir=".", output_file="compiled_project.py", exclude=None):
    """
    Combines all Python source files in the project into a single file.
    Each file is wrapped with header markers showing its path.
    """
    if exclude is None:
        exclude = ["venv", "__pycache__", "site-packages"]

    root_dir = Path(root_dir).resolve()
    output_path = root_dir / output_file

    with open(output_path, "w", encoding="utf-8") as outfile:
        outfile.write("# ================================================================\n")
        outfile.write("#   COMPILED PROJECT FILE\n")
        outfile.write("#   Automatically generated for unified analysis\n")
        outfile.write("# ================================================================\n\n")

        for folder, subdirs, files in os.walk(root_dir):
            # Skip unwanted directories
            if any(skip in folder for skip in exclude):
                continue

            for file in files:
                if not file.endswith(".py"):
                    continue
                file_path = Path(folder) / file

                outfile.write(f"\n\n# ================================================================\n")
                outfile.write(f"# ==== FILE: {file_path.relative_to(root_dir)} ====\n")
                outfile.write(f"# ================================================================\n\n")

                try:
                    with open(file_path, "r", encoding="utf-8") as infile:
                        content = infile.read()
                        outfile.write(content)
                        outfile.write("\n\n")
                except Exception as e:
                    outfile.write(f"# ⚠️ Could not read this file due to error: {e}\n\n")

        outfile.write("# ================================================================\n")
        outfile.write("# END OF COMPILED PROJECT\n")
        outfile.write("# ================================================================\n")

    print(f"✅ Project successfully compiled to: {output_path}")
    return str(output_path)


if __name__ == "__main__":
    # Change '.' to your project folder path if needed
    compile_project_to_single_file(root_dir="/Users/aryanmathur/Desktop/RDCP/take2")
