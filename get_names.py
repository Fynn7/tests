import requests
from bs4 import BeautifulSoup

# 目标网页的URL
url = 'https://moodle.uni-due.de/user/index.php?id=48250'  # 替换为实际的网址

# 发送HTTP请求获取网页内容
response = requests.get(url)
if response.status_code == 200:
    # 使用BeautifulSoup解析HTML内容
    soup = BeautifulSoup(response.content, 'html.parser')
    
    # 查找所有<td id='user-index-participants-48250_r0_c1'>元素
    td_elements = soup.find_all('td', id=lambda x: x and x.startswith('user-index-participants-48250_r0_c1'))
    
    # 遍历所有找到的元素，并选择末尾的最后一位数字是样本序号的元素
    for td in td_elements:
        text = td.get_text(strip=True)
        if text and text[-1].isdigit():
            print(text)
else:
    print(f"无法获取网页内容，状态码: {response.status_code}")