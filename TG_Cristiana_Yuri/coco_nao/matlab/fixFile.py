import re
import sys
with open(sys.argv[1], 'r') as file:
    content = file.read()

non_number = re.compile(r'[+ijkE(),]+')

ans = non_number.sub('', content)
ans = re.sub(' +',' ', ans)

with open(sys.argv[2], 'w') as file_:
    file_.write(ans)
