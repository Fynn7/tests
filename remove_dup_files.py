import os

def delete_files_ending_with_one(folder_path):
    print(f"Searching in: {folder_path}")
    try:
        for root, dirs, files in os.walk(folder_path):
            for file in files:
                # print(file.split('.'))
                if file.split('.')[-2].endswith('(1)'):
                    file_path = os.path.join(root, file)
                    os.remove(file_path)
                    print(f"Deleted: {file_path}")
    except Exception as e:
        print(f"Error: {e}")
# 指定要遍历的文件夹路径
folder_to_search = 'D:\\WeChat Files\\wxid_x7eaoeog6mx722\\FileStorage\\Video'
for root, dirs, files in os.walk(folder_to_search):
    for dir in dirs:
        # print(folder_to_search+"\\"+dir)
        folder_path = os.path.join(root, dir)
        # print(folder_path)
        delete_files_ending_with_one(folder_path)