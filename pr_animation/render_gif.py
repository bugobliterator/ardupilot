"""Render animation.html frame by frame in headless Chromium and write the GIF. Usage: render_gif.py <out.gif>"""
import io, pathlib, sys
from PIL import Image
from playwright.sync_api import sync_playwright
html = pathlib.Path(__file__).with_name('animation.html').resolve()
with sync_playwright() as p:
    browser = p.chromium.launch(executable_path=str(next(pathlib.Path.home().glob('Library/Caches/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-mac-arm64/chrome-headless-shell'))))
    page = browser.new_page(viewport={'width': 760, 'height': 560}, device_scale_factor=1)
    errors = []; page.on('pageerror', lambda e: errors.append(str(e)))
    page.goto(html.as_uri()); page.wait_for_timeout(500)
    if errors: raise SystemExit('page error: ' + errors[0])
    n = page.evaluate('window.FRAME_COUNT'); frames = []
    for i in range(n):
        page.evaluate(f'draw({i})')
        frames.append(Image.open(io.BytesIO(page.locator('#cv').screenshot())).convert('RGB'))
    browser.close()
pal = frames[len(frames) // 2].quantize(colors=128, method=Image.Quantize.MEDIANCUT)
q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
q[0].save(sys.argv[1], save_all=True, append_images=q[1:], duration=55, loop=0, optimize=True)
frames[len(frames) // 2].save(sys.argv[1].replace('.gif', '-mid.png')); frames[-1].save(sys.argv[1].replace('.gif', '-last.png'))
print(n, 'frames')
