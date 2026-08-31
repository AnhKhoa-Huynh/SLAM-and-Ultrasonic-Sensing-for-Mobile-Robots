from pathlib import Path
code_files = ["laptop_setup.py",
    "localisation_mapping_code.py",
    "exploration_strategy_code.py",
    "live_runtime_code.py",]

def main():
    folder = Path(__file__).resolve().parent
    shared = {"__name__": "mbot2_laptop_runtime", "__file__": str(folder / "laptop_run.py")}
    for filename in code_files:
        file_path = folder / filename
        code = compile(file_path.read_text(encoding="utf-8"), str(file_path), "exec")
        exec(code, shared)
    shared["main"]()

if __name__ == "__main__":
    main()
