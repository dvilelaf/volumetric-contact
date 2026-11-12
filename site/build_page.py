# /// script
# requires-python = ">=3.10"
# ///
"""Fills site/index.template.html: the landing page of the published site.

    uv run site/build_page.py --out build/site [--stats results/stats.json] [--figures DIR]

Everything variable is generated: the scenario list from the slim dataset's index.json, the results
table and the ranking from battery.csv next to it (tools/results_summary.py), the figures from the PNG/SVG files found in
--figures. Links to the paper and the repository appear only when their targets exist. The
repository address is the OWNER placeholder until scripts/set-owner.sh replaces it.
"""

import argparse
import json
import sys
from html import escape
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'tools'))
import results_summary

REPO = 'https://github.com/dvilelaf/volumetric-contact'


def scenario_list(index):
    items = []
    for name in sorted(index):
        cls = index[name].get('class') or ''
        badge = f'<b>{escape(cls)}</b>' if cls else ''
        items.append(f'      <li><a href="viewer/?scenario={escape(name)}">{badge}{escape(name)}</a></li>')
    return '\n'.join(items)


FIGURE_CAPTIONS = {
    'heatmap_A': 'Class A: error of every method (rows) on every scenario (columns), log colour scale; grey is a failed run.',
    'heatmap_B': 'Class B: error of every method (rows) on every scenario (columns), log colour scale; grey is a failed run.',
    'heatmap_C': 'Class C: error of every method (rows) on every scenario (columns), log colour scale; grey is a failed run.',
    'profile_AB': 'Performance profile on classes A and B: the share of scenarios each method solves within a factor of the best error; higher and further left is better.',
    'profile_C': 'Performance profile on class C: the share of scenarios each method solves within a factor of the best error; higher and further left is better.',
    'convergence_dt': 'Median error as the time step shrinks, relative to the coarsest step; a falling curve converges.',
    'convergence_mesh': 'Median error as the mesh is refined, relative to the coarsest mesh; a falling curve converges.',
    'work_precision': 'Median error against median wall time per run as the time step varies; lower left is better.',
    'forest_AB': 'Paired error ratios with bootstrap intervals on classes A and B; below 1 favours the first method of the pair.',
    'ablation_triangulation': 'Error of the friction torque on one flat face as its triangulation is refined, for each friction quadrature (left, right) and each triangulation.',
}


def figures(folder):
    files = sorted(p for p in (Path(folder).glob('*') if folder and Path(folder).is_dir() else [])
                   if p.suffix in ('.png', '.svg'))
    if not files:
        return ''

    def caption(p):
        return FIGURE_CAPTIONS.get(p.stem, p.stem.replace('-', ' ').replace('_', ' '))

    items = ''.join(f'    <figure><img src="figures/{escape(p.name)}" alt="{escape(caption(p))}">'
                    f'<figcaption>{escape(caption(p))}</figcaption></figure>\n'
                    for p in files)
    return f'  <section aria-labelledby="fig-h">\n    <h2 id="fig-h">Figures</h2>\n{items}  </section>\n'


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out', required=True)
    p.add_argument('--stats', default='results/stats.json')
    p.add_argument('--figures', default='results/figures')
    a = p.parse_args()
    out = Path(a.out)
    index = json.loads((out / 'data/traj/index.json').read_text())
    index = index.get('scenarios', index)
    links = ['<a class="primary" href="viewer/">Viewer</a>']
    if (out / 'paper.pdf').exists():
        links.append('<a href="paper.pdf">Paper (PDF)</a>')
    if (out / 'supplement.pdf').exists():
        links.append('<a href="supplement.pdf">Supplement (PDF)</a>')
    links += [f'<a href="{REPO}">Repository</a>', f'<a href="{REPO}/blob/main/docs/architecture.md">Docs</a>']
    page = (HERE / 'index.template.html').read_text()
    fields = {
        'scenarios': str(len(index)),
        'links': '\n    '.join(links),
        'scenario_list': scenario_list(index),
        'results': results_summary.html(Path(a.stats).parent / 'battery.csv'),
        'ranking': results_summary.ranking_html(Path(a.stats).parent / 'battery.csv', Path(a.stats).parent / 'timing.csv'),
        'figures': figures(out / 'figures'),
        'repo': REPO,
        'footer': f'<a href="{REPO}">Source and data</a>.',
    }
    for key, value in fields.items():
        page = page.replace('{{' + key + '}}', value)
    (out / 'index.html').write_text(page)


if __name__ == '__main__':
    main()
