"""Render an animation page frame by frame in headless Chromium and write a GIF.
Usage: render_gif.py <page.html> <out.gif> [query]   e.g. render_gif.py turn.html turn.gif 'el=14'"""
import io, pathlib, sys
from PIL import Image
from playwright.sync_api import sync_playwright
page_path = pathlib.Path(__file__).with_name(sys.argv[1]).resolve(); out = sys.argv[2]
query = ('?' + sys.argv[3]) if len(sys.argv) > 3 else ''
exe = str(next(pathlib.Path.home().glob('Library/Caches/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-mac-arm64/chrome-headless-shell')))
with sync_playwright() as p:
    browser = p.chromium.launch(executable_path=exe)
    page = browser.new_page(viewport={'width': 1200, 'height': 700}, device_scale_factor=1)
    errors = []; page.on('pageerror', lambda e: errors.append(str(e)))
    page.goto(page_path.as_uri() + query); page.wait_for_timeout(500)
    if errors: raise SystemExit('page error: ' + errors[0])
    n = page.evaluate('window.FRAME_COUNT'); frames = []
    for i in range(n):
        page.evaluate(f'draw({i})')
        frames.append(Image.open(io.BytesIO(page.locator('#stage').screenshot())).convert('RGB'))
    browser.close()
pal = frames[len(frames) // 2].quantize(colors=128, method=Image.Quantize.MEDIANCUT)
q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
q[0].save(out, save_all=True, append_images=q[1:], duration=55, loop=0, optimize=True)
for k, i in (('first', 0), ('mid', n // 2), ('last', n - 1)): frames[i].save(out.replace('.gif', f'-{k}.png'))
print(n, 'frames')
