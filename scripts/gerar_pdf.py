"""Gera RELATORIO.pdf a partir de RELATORIO_RASCUNHO.md.

Markdown -> HTML (o diagrama Mermaid é desenhado pelo mermaid.js, baixado da CDN) -> PDF pelo
Chromium headless. Tira o "(RASCUNHO)" do título e a seção de pendências do fim.

Requisitos: python3 com markdown-it-py e o navegador chromium; acesso à internet (mermaid.js).
Uso, da raiz do repositório:  python3 scripts/gerar_pdf.py
"""
import re
import subprocess
import sys
from html import escape
from pathlib import Path

from markdown_it import MarkdownIt

import tempfile

REPO = Path(__file__).resolve().parent.parent
OUT_HTML = Path(tempfile.mkdtemp()) / "relatorio.html"
OUT_PDF = (Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "RELATORIO.pdf").resolve()

src = (REPO / "RELATORIO_RASCUNHO.md").read_text(encoding="utf-8")
src = src.replace(" (RASCUNHO)", "", 1)
src = re.split(r"\n---\s*\n+## Pendências antes de converter para PDF", src)[0].rstrip() + "\n"

md = MarkdownIt("commonmark", {"html": False, "linkify": False}).enable(["table", "strikethrough"])


def fence(self, tokens, idx, options, env):
    tok = tokens[idx]
    if tok.info.strip() == "mermaid":
        return '<div class="diagram"><pre class="mermaid">%s</pre></div>\n' % escape(tok.content)
    return self.fence(tokens, idx, options, env)


md.add_render_rule("fence", fence)
body = md.render(src)
# Figuras: legenda a partir do alt
body = re.sub(r'<p><img src="([^"]+)" alt="([^"]*)" ?/?></p>',
              r'<figure><img src="\1" alt="\2"><figcaption>\2</figcaption></figure>', body)
# Links nus viram clicáveis
body = re.sub(r'(?<![">])(https://github\.com/[^\s<,)]+?)(?=[.,]?(\s|<))', r'<a href="\1">\1</a>', body)

css = """
@page { size: A4; margin: 18mm 17mm 18mm 17mm;
  @bottom-center { content: counter(page); } }
html { -webkit-print-color-adjust: exact; print-color-adjust: exact; }
body { font-family: "DejaVu Serif", "Liberation Serif", Georgia, serif; font-size: 10.3pt; line-height: 1.42;
  color: #111; background: #fff; margin: 0; }
h1 { font-size: 19pt; margin: 0 0 6pt; line-height: 1.2; }
h2 { font-size: 14pt; margin: 18pt 0 6pt; border-bottom: 1px solid #bbb; padding-bottom: 2pt; break-after: avoid; }
h3 { font-size: 11.5pt; margin: 12pt 0 4pt; break-after: avoid; }
p, li { text-align: justify; hyphens: auto; }
ul, ol { padding-left: 18pt; margin: 4pt 0; }
li { margin: 2pt 0; }
code { font-family: "DejaVu Sans Mono", monospace; font-size: 8.6pt; background: #f2f2f0; padding: 0 2pt;
  border-radius: 2pt; }
pre { background: #f6f6f4; border: 1px solid #ddd; border-radius: 3pt; padding: 5pt 6pt; font-size: 7.2pt;
  line-height: 1.35; white-space: pre-wrap; break-inside: avoid; }
pre code { background: none; padding: 0; font-size: inherit; }
table { border-collapse: collapse; margin: 6pt 0 10pt; font-size: 8.8pt; break-inside: avoid; }
th, td { border: 1px solid #c8c8c8; padding: 3pt 5pt; vertical-align: top; text-align: left; }
th { background: #eeeeec; }
blockquote { border-left: 3px solid #ccc; margin: 6pt 0; padding: 2pt 8pt; color: #444; }
hr { display: none; }
figure { margin: 8pt 0 12pt; text-align: center; break-inside: avoid; }
figure img { max-width: 100%; max-height: 225mm; }
figcaption { font-size: 9pt; color: #444; margin-top: 3pt; }
.diagram { text-align: center; margin: 8pt 0; break-inside: avoid; }
pre.mermaid { background: none; border: none; }
a { color: #1a4f9c; text-decoration: none; word-break: break-all; }
"""

html = f"""<!doctype html>
<html lang="pt-BR"><head><meta charset="utf-8">
<base href="{REPO.as_uri()}/">
<title>Relatório 5 — ROS e navegação</title>
<style>{css}</style>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js"></script>
<script>mermaid.initialize({{startOnLoad: true, theme: "neutral", flowchart: {{htmlLabels: true}}}});</script>
</head><body>
{body}
</body></html>
"""
OUT_HTML.write_text(html, encoding="utf-8")
subprocess.run([
    "chromium", "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
    "--allow-file-access-from-files", "--virtual-time-budget=15000",
    f"--print-to-pdf={OUT_PDF}", OUT_HTML.as_uri(),
], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print("ok", OUT_PDF)
