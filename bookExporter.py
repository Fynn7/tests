import pandas as pd

def txt_to_xlsx(txt_file, xlsx_file):
    with open(txt_file, 'r',encoding="cp1252") as f:
        lines=f.readlines()
    
    lines = [line.strip().split('\t') for line in lines]
    df = pd.DataFrame(lines)
    df.to_excel(xlsx_file, index=False, header=False)

if __name__ == '__main__':
    txt_to_xlsx('alleBuecher.txt', 'alleBuecher.xlsx')