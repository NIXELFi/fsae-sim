import json,base64,sys
d=json.load(open(sys.argv[1],encoding='utf-8'))
c=d['content']
try: b=base64.b64decode(c,validate=True)
except Exception: b=c.encode('utf-8')
open(sys.argv[2],'wb').write(b); print(d.get('title'),len(b))
