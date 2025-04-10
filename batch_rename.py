import os

def rename_hdf5_files(directory):
    # 获取目录下所有的 .hdf5 文件
    files = [f for f in os.listdir(directory) if f.endswith('.hdf5')]
    files.sort()  # 对文件列表进行排序
    index = 21
    for filename in files:
        new_name = f'episode_{index}.hdf5'
        new_path = os.path.join(directory, new_name)

        # 检查新文件名是否已存在
        if os.path.exists(new_path):
            print(f'文件 {new_name} 已存在，跳过重命名 {filename}')
        else:
            old_path = os.path.join(directory, filename)
            os.rename(old_path, new_path)
            print(f'已将 {filename} 重命名为 {new_name}')
            index += 1
# 使用示例
directory = '/workspace/hdf5_file'  # 替换为您的目标目录路径
rename_hdf5_files(directory)
