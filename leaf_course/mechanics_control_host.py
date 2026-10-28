"""Mac-local central console, forwarding only defined controls to Ubuntu via SSH."""
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from urllib.request import Request,urlopen
from urllib.error import URLError
from pathlib import Path
import ast
P=Path(__file__).resolve().parent
class H(BaseHTTPRequestHandler):
 def log_message(self,*args):pass
 def send(self,code,body,kind):
  self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
 def do_GET(self):
  if self.path=='/':self.send(200,(P/'console.html').read_bytes(),'text/html; charset=utf-8');return
  if self.path.split('?')[0] not in ['/state','/camera.jpg','/telemetry','/frames/left.jpg','/frames/right.jpg','/frames/depth.jpg']:self.send(404,b'Not found','text/plain');return
  self.proxy('GET')
 def do_POST(self):
  if self.path not in ['/reset','/start'] or self.headers.get('X-Leaf-Demo')!='1':self.send(403,b'Forbidden','text/plain');return
  self.proxy('POST')
 def proxy(self,method):
  try:
   port=18083 if self.path.startswith(('/telemetry','/frames/')) else 18082
   req=Request('http://127.0.0.1:'+str(port)+self.path,data=b'' if method=='POST' else None,headers={'X-Leaf-Demo':'1'},method=method)
   with urlopen(req,timeout=8) as r:self.send(r.status,r.read(),r.headers.get('Content-Type','application/json'))
  except (URLError,TimeoutError):self.send(503,b'{"error":"Ubuntu connection unavailable"}','application/json')
print('Mac central console: http://127.0.0.1:8082',flush=True)
ThreadingHTTPServer(('127.0.0.1',8082),H).serve_forever()
