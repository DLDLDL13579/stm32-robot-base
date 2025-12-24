import os

def list_c_h_files_to_txt(root_dir, output_file, exclude_dirs=None):
    """
    将指定目录及其子目录下所有.c和.h文件的内容按指定格式写入文本文件，并可排除特定目录。

    Args:
        root_dir (str): 要遍历的根目录路径
        output_file (str): 输出文本文件的路径
        exclude_dirs (list, optional): 需要排除的目录名称列表。默认为 None，即排除 ['CMSIS', 'FWLib', 'OBJ']
    """
    # 设置默认要排除的目录
    if exclude_dirs is None:
        exclude_dirs = ['CMSIS', 'FWLib', 'OBJ']
    
    # 检查根目录是否存在
    if not os.path.exists(root_dir):
        print(f"错误：目录 '{root_dir}' 不存在")
        return False
    
    try:
        with open(output_file, 'w', encoding='utf-8') as outfile:
            # 使用os.walk递归遍历目录
            for root, dirs, files in os.walk(root_dir):
                # ！关键改进：在遍历当前目录的子目录前，从dirs列表中移除要排除的目录
                # 这将阻止os.walk进入这些目录，实现排除
                dirs[:] = [d for d in dirs if d not in exclude_dirs]
                
                for file in files:
                    # 检查文件扩展名是否为.c或.h
                    if file.endswith(('.c', '.h')):
                        file_path = os.path.join(root, file)
                        try:
                            # 读取文件内容，尝试不同的编码
                            with open(file_path, 'r', encoding='utf-8') as infile:
                                content = infile.read()
                        except UnicodeDecodeError:
                            try:
                                # 如果UTF-8解码失败，尝试GBK编码
                                with open(file_path, 'r', encoding='gbk') as infile:
                                    content = infile.read()
                            except Exception as e:
                                content = f"【无法读取文件内容，错误：{str(e)}】"
                        except Exception as e:
                            content = f"【读取文件时出错：{str(e)}】"
                        
                        # 写入文件名和内容到输出文件
                        outfile.write(f"文件名: {file}\n")
                        outfile.write(f"文件路径: {file_path}\n")
                        outfile.write("文件内容:\n")
                        outfile.write(content)
                        outfile.write("\n" + "="*50 + "\n\n")
                        
            print(f"成功处理完成！结果已保存到: {output_file}")
            return True
            
    except Exception as e:
        print(f"处理过程中发生错误: {str(e)}")
        return False

if __name__ == "__main__":
    # 设置要遍历的目录路径
    target_directory = input("请输入要遍历的目录路径（留空则使用当前目录）: ").strip()
    if not target_directory:
        target_directory = os.getcwd()
    
    # 设置输出文件路径
    output_filename = "c_h_files_content.txt"
    
    # ！执行处理，并指定要排除的目录
    list_c_h_files_to_txt(target_directory, output_filename, exclude_dirs=['CMSIS', 'FWLib', 'OBJ'])