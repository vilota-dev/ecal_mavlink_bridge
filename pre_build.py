#!/usr/bin/env python3

import os
import subprocess
import sys
import argparse

thirdparty_modules = [
    "MAVSDK"
]

CACHE_COMMITS_PATH = "cache/commits.txt"

def run_command(cmd_list, cwd=None):
    print(f"Running command: {' '.join(cmd_list)}")
    try:
        subprocess.run(cmd_list, check=True, cwd=cwd)
    except subprocess.CalledProcessError as e:
        print(f"Error executing {' '.join(e.cmd)}")
        sys.exit(1)

def get_submodule_hashes():
    result = subprocess.run(
        ["git", "submodule", "status"],
        capture_output=True,
        text=True,
        check=True
    )
    lines = result.stdout.strip().splitlines()
    hashes = {}
    for line in lines:
        if line:
            parts = line.strip().split()
            hash_val = parts[0].lstrip("-+")
            path = parts[1]
            hashes[path] = hash_val
    return hashes

def read_saved_hashes():
    if not os.path.isfile(CACHE_COMMITS_PATH):
        return {}
    with open(CACHE_COMMITS_PATH, "r") as f:
        lines = f.readlines()
    saved = {}
    for line in lines:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) == 2:
            saved[parts[0]] = parts[1]
    return saved

def save_hashes(hashes):
    os.makedirs(os.path.dirname(CACHE_COMMITS_PATH), exist_ok=True)
    with open(CACHE_COMMITS_PATH, "w") as f:
        for path, hash_val in hashes.items():
            f.write(f"{path} {hash_val}\n")

def should_build(module_name, module_path, current_hashes, saved_hashes, project_root_dir):
    install_dir = os.path.join(project_root_dir, "cache", f"{module_name}-install")
    # Build if submodule hash changed or install directory missing
    if current_hashes.get(module_path) != saved_hashes.get(module_path):
        return True
    if not os.path.isdir(install_dir):
        return True
    return False

def build_mavsdk(project_root_dir, nthread=4):
    module_name = "MAVSDK"
    thirdparty_dir = os.path.join(project_root_dir, "thirdparty", module_name)
    build_dir = os.path.join(project_root_dir, "cache", f"{module_name}-build")
    install_dir = os.path.join(project_root_dir, "cache", f"{module_name}-install")

    if os.path.exists(build_dir):
        run_command(["rm", "-rf", build_dir])
    if os.path.exists(install_dir):
        run_command(["rm", "-rf", install_dir])

    os.makedirs(build_dir, exist_ok=True)

    cmake_cmd = [
        "cmake",
        thirdparty_dir,
        f"-DCMAKE_INSTALL_PREFIX={install_dir}",
        "-DBUILD_MAVSDK_SERVER=OFF",
        "-DBUILD_SHARED_LIBS=ON",
        "-DCMAKE_BUILD_TYPE=RelWithDebInfo"
    ]

    run_command(cmake_cmd, cwd=build_dir)
    run_command(["make", f"-j{nthread}"], cwd=build_dir)
    run_command(["cmake", "--install", "."], cwd=build_dir)

    print(f"{module_name} built and installed to: {install_dir}")

def build_module(module_name, project_root_dir, nthread=4):
    if module_name == "MAVSDK":
        build_mavsdk(project_root_dir, nthread)
    else:
        print(f"No build instructions for module: {module_name}")

def main():
    parser = argparse.ArgumentParser(description="Build thirdparty modules")
    parser.add_argument(
        "--project_root",
        type=str,
        default=os.path.abspath(os.path.dirname(__file__)),
        help="Root directory of the project"
    )
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=os.cpu_count() or 4,
        help="Number of parallel jobs to use when building"
    )
    args = parser.parse_args()

    project_root_dir = args.project_root
    nthread = args.jobs

    print(f"Using project root: {project_root_dir}")
    print(f"Using {nthread} parallel jobs")

    current_hashes = get_submodule_hashes()
    saved_hashes = read_saved_hashes()

    for module_name in thirdparty_modules:
        module_path = os.path.join("thirdparty", module_name)
        if should_build(module_name, module_path, current_hashes, saved_hashes, project_root_dir):
            print(f"\n[Info] Building {module_name} because submodule changed or not built yet.")
            # Update submodule
            run_command([
                "git", "submodule", "update", "--init", "--recursive", "--depth=1", module_path
            ], cwd=project_root_dir)
            # Build the module
            build_module(module_name, project_root_dir, nthread)
        else:
            print(f"\n[Info] Skipping {module_name}: submodule unchanged.")

    save_hashes(current_hashes)
    print("\nAll requested modules are up to date and built.")

if __name__ == "__main__":
    main()
