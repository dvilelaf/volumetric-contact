import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import results_summary as rs

HEAD = 'engine,contact,scenario,variant,status,error,realtime_ratio,wall_s,steps,dt,segments,stiffness_scale,class\n'


def battery(tmp_path):
    path = tmp_path / 'battery.csv'
    path.write_text(HEAD + ''.join(f'{line},0.1,1,1,0.001,0,1,{cls}\n' for line, cls in (
        ('chrono,gonthier,s1,base,ok,0.02', 'A'),
        ('chrono,gonthier,s2,base,ok,0.06', 'A'),
        ('drake,gonthier,s1,base,ok,0.04', 'A'),
        ('drake,gonthier,s2,base,timeout,', 'A'),          # no value: counts as 100 %
        ('chrono,native-nsc,s3,base,ok,0.2', 'B'),
        ('chrono,native-nsc,s4,base,ok,0.3', 'C'),         # class C: Winkler-type methods only
        ('chrono,native,s1,base,ok,0.05', 'A'),
        ('chrono,native,s2,base,ok,7.0', 'A'),             # above the cap: counts as 100 %
        ('chrono,native,s5,base,nan,', 'A'),
        ('drake,native,s1,base,ok,0.01', 'A'),
        ('drake,native,s4,base,ok,0.3', 'C'),
        ('chrono,gonthier,s1,dt2,ok,0.9', 'A'),            # not the base variant: ignored
        ('other,gonthier,s1,base,ok,0.0', 'A'),            # not an engine: ignored
    )))
    return path


def test_table_pools_the_engines_of_a_method(tmp_path):
    table = rs.table(battery(tmp_path))
    assert list(table) == ['gonthier', 'Drake hydroelastic', 'Chrono SMC', 'Chrono NSC']
    assert table['gonthier']['A'] == (0.05, 0.5, 4)        # median of 0.02, 0.04, 0.06 and the capped 1.0


def test_markdown_and_html_name_every_method(tmp_path):
    path = battery(tmp_path)
    assert '| `gonthier` | 5.0 % (50 %) | — | — |' in rs.markdown(path)
    assert 'count as 100 %' in rs.markdown(path) and 'count as 100 %' in rs.html(path)
    assert '<th scope="row">Chrono NSC</th>' in rs.html(path)


def test_native_models_of_different_engines_are_not_pooled(tmp_path):
    table = rs.table(battery(tmp_path))
    assert table['Drake hydroelastic'] == {'A': (0.01, 1.0, 1), 'C': (0.3, 0.0, 1)}
    assert table['Chrono SMC']['A'] == (1.0, 1 / 3, 3)     # 0.05, the capped 7.0 and the nan run
    assert table['Chrono NSC'] == {'B': (0.2, 0.0, 1)}
    assert '| Drake hydroelastic | 1.0 % (100 %) | — | 30.0 % (0 %) |' in rs.markdown(battery(tmp_path))


def test_table_and_ranking_share_one_median(tmp_path):
    path = battery(tmp_path)
    smc = next(r for r in rs.ranking(path, tmp_path / 'none.csv') if r['name'] == 'Chrono SMC')
    assert smc['median_A'] == rs.table(path)['Chrono SMC']['A'][0] == 1.0


def test_absent_battery_says_so(tmp_path):
    assert rs.table(tmp_path / 'none.csv') is None
    assert 'not in this checkout' in rs.markdown(tmp_path / 'none.csv')
    assert 'not in this checkout' in rs.html(tmp_path / 'none.csv')


def test_readme_block_is_rewritten_between_the_markers(tmp_path):
    readme = tmp_path / 'README.md'
    readme.write_text(f'before\n{rs.BEGIN}\nold\n{rs.END}\nafter\n')
    rs.readme(battery(tmp_path), readme)
    text = readme.read_text()
    assert text.startswith('before\n') and text.endswith('\nafter\n')
    assert 'old' not in text and '`gonthier`' in text


def test_ranking_orders_by_accuracy_and_marks_the_fastest(tmp_path):
    head = 'engine,contact,scenario,variant,status,error,realtime_ratio,wall_s,steps,dt,segments,stiffness_scale,class\n'
    battery = tmp_path / 'battery.csv'
    battery.write_text(head
                       + 'chrono,distributed,s1,base,ok,0.01,0.1,1,1,0.001,0,1,A\n'
                       + 'chrono,distributed,s2,base,ok,0.02,0.1,1,1,0.001,0,1,B\n'
                       + 'chrono,native,s1,base,ok,0.2,0.1,1,1,0.001,0,1,A\n'
                       + 'chrono,native,s2,base,nan,,0.1,1,1,0.001,0,1,B\n'
                       + 'chrono,distributed,s3,base,ok,9.0,0.1,1,1,0.001,0,1,C\n'       # class C: left out
                       + 'chrono,distributed-vertex,s1,base,ok,0.0,0.1,1,1,0.001,0,1,A\n'  # a variant: left out
                       + 'chrono,distributed,s1,dt2,ok,0.5,0.1,1,1,0.002,0,1,A\n'        # not the base variant
                       + 'chrono,gonthier-thesis,s1,base,ok,0.3,0.1,1,1,0.001,0,1,A\n')
    timing = tmp_path / 'timing.csv'
    timing.write_text(head
                      + 'chrono,distributed,s1,base,ok,0.01,0.10,1,1,0.001,0,1,A\n'
                      + 'chrono,distributed,s1,base,ok,0.01,0.30,1,1,0.001,0,1,A\n'
                      + 'chrono,native,s1,base,ok,0.2,0.01,1,1,0.001,0,1,A\n')
    rows = rs.ranking(battery, timing)
    assert [r['name'] for r in rows] == ['Distributed law', 'Gonthier law (2018 thesis variant)', 'Chrono SMC']
    rows = [r for r in rows if r['contact'] != 'gonthier-thesis']   # the thesis variant is its own contender
    assert rows[0]['within'] == 1.0 and rows[0]['median'] == 0.015 and abs(rows[0]['speed'] - 5.0) < 1e-12
    assert rows[1]['within'] == 0.0 and rows[1]['median'] == 0.6   # the nan run counts as 100 %
    assert 'front' not in rows[0]
    assert rs.balance(rows) == [rows[0]]                           # Chrono SMC is faster, with 0 % within 5 %
    page = rs.ranking_html(battery, timing)
    assert 'Chrono SMC: ×100 real time' in page and 'Distributed law of this library (on any of the engines)' in page
    assert page.count('<i class="mark">best balance</i>') == 1 and 'best trade-off' not in page
    assert '<b>Best balance</b><span>Distributed law of this library on Chrono: ×5 real time' in page
    assert 'Best balance: the fastest model with at least 60 % of the scenarios within 5 %.' in page
    assert 'Scenarios within 5 % error<span class="sub">share, higher is better</span>' in page
    assert '<span class="sub">× real time, higher is better</span>' in page and 'title="Simulated time divided' in page
    assert 'not in this checkout' in rs.ranking_html(tmp_path / 'none.csv', tmp_path / 'none.csv')
