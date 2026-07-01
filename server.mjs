import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { extname, join, normalize, resolve } from 'node:path';

const root = process.cwd();
const port = Number(process.env.PORT || 5173);

const mime = new Map([
  ['.html', 'text/html; charset=utf-8'],
  ['.js', 'text/javascript; charset=utf-8'],
  ['.css', 'text/css; charset=utf-8'],
  ['.json', 'application/json; charset=utf-8'],
  ['.fbx', 'application/octet-stream'],
  ['.wasm', 'application/wasm'],
]);

function resolvePath(url) {
  const safePath = decodeURIComponent(new URL(url, 'http://local').pathname);
  const target = safePath === '/' ? '/index.html' : safePath;
  const filePath = resolve(root, `.${normalize(target)}`);
  if (!filePath.startsWith(root)) return null;
  return filePath;
}

createServer(async (req, res) => {
  const filePath = resolvePath(req.url || '/');
  if (!filePath) {
    res.writeHead(403);
    res.end('Forbidden');
    return;
  }

  try {
    const info = await stat(filePath);
    const finalPath = info.isDirectory() ? join(filePath, 'index.html') : filePath;
    const data = await readFile(finalPath);
    res.writeHead(200, {
      'Content-Type': mime.get(extname(finalPath).toLowerCase()) || 'application/octet-stream',
      'Cache-Control': 'no-store',
    });
    res.end(data);
  } catch {
    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end('Not found');
  }
}).listen(port, '127.0.0.1', () => {
  console.log(`Fracture demo: http://127.0.0.1:${port}/`);
});
