#!/usr/bin/python3

import os
import subprocess
import sys
import argparse

# List of all thirdparty modules you want to manage
thirdparty_modules = [
    "MAVSDK"
]

def run_command(cmd_list, cwd=None):
    print(f"Running command: {' '.join(cmd_list)}")
    try:
        subprocess.run(cmd_list, check=True, cwd=cwd)
    except subprocess.CalledProcessError as e:
        print(f"Error executing {' '.join(e.cmd)}: {e.stderr if e.stderr else e}")
        sys.exit(1)

def build_mavsdk(project_root_dir, nthread=4):
    module_name = "MAVSDK"
    thirdparty_dir = os.path.join(project_root_dir, "thirdparty", module_name)
    build_dir = os.path.join(project_root_dir, "cache", f"{module_name}-build")
    install_dir = os.path.join(project_root_dir, "cache", f"{module_name}-install")

    run_command(["rm", "-rf", build_dir])
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
    return install_dir

def build_module(module_name, project_root_dir, nthread=4):
    if module_name == "MAVSDK":
        build_mavsdk(project_root_dir, nthread)
    else:
        print(f"No build instructions for module: {module_name}")

# --- Git helper functions to get submodule commits etc. ---

def read_gitmodules():
    submodule_paths = []
    with open(".gitmodules", "r") as file:
        lines = file.readlines()
        for line in lines:
            if line.strip().startswith("path ="):
                submodule_path = line.split("=")[1].strip()
                submodule_paths.append(submodule_path)
    return submodule_paths

def get_submodule_commit_hash(submodule_path):
    result = subprocess.run(
        ["git", "submodule", "status", submodule_path], capture_output=True, text=True
    )
    if result.returncode == 0:
        output = result.stdout[1:]  # remove first char (status symbol)
        parts = output.split()
        if len(parts) >= 2:
            return parts[0]
    return None

def get_current_commit_hashes():
    submodule_commits = {}
    submodule_paths = read_gitmodules()
    for path in submodule_paths:
        commit_hash = get_submodule_commit_hash(path)
        if commit_hash:
            submodule_name = path.replace("thirdparty/", "")
            submodule_commits[submodule_name] = commit_hash
    return submodule_commits

def read_commits_txt(commits_txt_path):
    commits_dict = {}
    try:
        with open(commits_txt_path, "r") as f:
            for line in f:
                if line.strip():
                    key, val = line.strip().split()
                    commits_dict[key] = val
    except FileNotFoundError:
        print(f"commits.txt not found at '{commits_txt_path}'")
    return commits_dict

def write_commits_to_file(submodule_commits, file_path):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w") as file:
        for submodule, commit_hash in submodule_commits.items():
            file.write(f"{submodule} {commit_hash}\n")

def compare_commit_hashes(commits_dict, current_commit_hashes):
    prebuild_package_list = []
    for submodule, commit_hash in current_commit_hashes.items():
        if commits_dict.get(submodule) != commit_hash:
            print(f"[Debug] {submodule} {commits_dict.get(submodule)} vs {commit_hash}")
            prebuild_package_list.append(submodule)
    return prebuild_package_list

def main():
    parser = argparse.ArgumentParser(description="Build thirdparty modules with smart updates")
    parser.add_argument(
        "-s", action="store_true", help="Use full CPU cores (nthread = cpu_count). Otherwise half cores"
    )
    args = parser.parse_args()

    project_root_dir = os.path.abspath(os.path.dirname(__file__))

    if args.s:
        nthread = os.cpu_count()
    else:
        nthread = max(1, os.cpu_count() // 2)

    commits_txt_path = os.path.join(project_root_dir, "cache", "commits.txt")
    commits_dict = read_commits_txt(commits_txt_path)
    current_commit_hashes = get_current_commit_hashes()
    prebuild_package_list = compare_commit_hashes(commits_dict, current_commit_hashes)

    if prebuild_package_list:
        for module_name in prebuild_package_list:
            print(f"[Debug] Updating {module_name} in thirdparty folder")
            update_submodule_command = [
                "git",
                "submodule",
                "update",
                "--init",
                "--recursive",
                "--depth=1",
                f"thirdparty/{module_name}",
            ]
            subprocess.run(update_submodule_command, check=True, cwd=project_root_dir)
            build_module(module_name, project_root_dir, nthread)

            # Update commits.txt after each pre-build is done
            os.chdir(project_root_dir)
            write_commits_to_file(get_current_commit_hashes(), commits_txt_path)
    else:
        print("All the thirdparties have correct versions")

    print("All modules have been successfully built.")

if __name__ == "__main__":
    main()
