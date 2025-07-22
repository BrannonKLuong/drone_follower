import os
SKIP_NAMES = {'.venv', '.git', 'node_modules', 
              '__pycache__', '.angular', '.bundle', 
              'android', 'ios', '__tests__', 'temp_uploads', 
              'Include', 'Lib', 'Scripts', 'Lib64',}

def print_tree(path, prefix=''):
    entries = sorted(os.listdir(path))
    # filter out anything in SKIP_NAMES up‐front
    entries = [
        e for e in entries
        if e not in SKIP_NAMES
    ]
    for idx, name in enumerate(entries):
        full = os.path.join(path, name)

        # Skip downloaded JS files (or other extensions)
        if os.path.isfile(full) and (name.endswith('.js') or name.endswith('.js.map')):
            continue

        connector = '└── ' if idx == len(entries)-1 else '├── '
        print(prefix + connector + name)
        
        if os.path.isdir(full):
            extension = '    ' if idx == len(entries)-1 else '│   '
            print_tree(full, prefix + extension)

if __name__ == '__main__':
    root = r'C:\Users\Brannon Luong\Desktop\Drone'
    print(root)
    print_tree(root)
