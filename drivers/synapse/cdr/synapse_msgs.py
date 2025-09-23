import os
import json
import subprocess
import sys

# Get the absolute path of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Empy template filenames with absolute paths
TEMPLATE_C = os.path.join(script_dir, 'synapse_msgs.c.em')
TEMPLATE_H = os.path.join(script_dir, 'synapse_msgs.h.em')

# Empy executable (assume 'empy' is in PATH)
EMPY_EXEC = 'empy'

def main(json_folder, out_folder):
    # Recursively find all JSON files
    json_files = []
    for root, dirs, files in os.walk(json_folder):
        for file in files:
            if file.endswith('.json'):
                json_files.append(os.path.join(root, file))

    if not json_files:
        print(f"No JSON files found in {json_folder}")
        return

    # Merge all JSON contents into a list
    merged_data = []
    for json_path in json_files:
        with open(json_path, 'r') as f:
            data = json.load(f)
            merged_data.append(data)

    # Output files in the root of the input folder
    out_c = os.path.join(out_folder, "synapse_msgs.c")
    out_h = os.path.join(out_folder, "synapse_msgs.h")

    # Pass the merged data as a variable to Empy
    empy_vars = [f"-Djson_data={json.dumps(merged_data)}"]

    # Generate .c file
    with open(out_c, 'w') as fout:
        subprocess.run(
            [EMPY_EXEC, *empy_vars, TEMPLATE_C],
            stdout=fout,
            check=True
        )

    # Generate .h file
    with open(out_h, 'w') as fout:
        subprocess.run(
            [EMPY_EXEC, *empy_vars, TEMPLATE_H],
            stdout=fout,
            check=True
        )

    print(f"Generated: {out_c}, {out_h}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"Usage: python {sys.argv[0]} <json_folder> <out_folder>")
        sys.exit(1)
    main(sys.argv[1],sys.argv[2])
