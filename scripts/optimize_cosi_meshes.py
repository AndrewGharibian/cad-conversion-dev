"""Repair tiny facets and produce conservative Geomega mesh variants.

Dependencies: numpy, scipy, trimesh, fast-simplification, rtree.
Run with a Python environment containing those packages from the repo root.
Original files are never overwritten. Distances and areas use centimeters.
"""

import argparse
import csv
import json
import re
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def read_mesh(path):
    text = path.read_text()
    vertices, faces = [], []
    for line in text.splitlines():
        tokens = line.split()
        if tokens and tokens[0].endswith('.Vertex'):
            vertices.append([float(x) for x in tokens[1:]])
        elif tokens and tokens[0].endswith('.Facet'):
            faces.append([int(x) for x in tokens[1:]])
    return text, np.array(vertices), np.array(faces, dtype=np.int64)


def compact(v, f):
    used, inverse = np.unique(f, return_inverse=True)
    return v[used], inverse.reshape((-1, 3))


def areas(v, f):
    return np.linalg.norm(np.cross(v[f[:, 1]] - v[f[:, 0]],
                                   v[f[:, 2]] - v[f[:, 0]]), axis=1) * .5


def topology(v, f):
    if not np.isfinite(v).all() or f.min() < 0 or f.max() >= len(v):
        raise ValueError('Invalid coordinates or indices')
    edges = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    ordered = np.sort(edges, axis=1)
    unique, inverse, counts = np.unique(ordered, axis=0, return_inverse=True,
                                       return_counts=True)
    signs = np.where(edges[:, 0] < edges[:, 1], 1, -1)
    balance = np.bincount(inverse, weights=signs)
    if np.any(counts != 2) or np.any(balance != 0):
        raise ValueError('Mesh is open, non-manifold, or inconsistently wound')
    if len(np.unique(np.sort(f, axis=1), axis=0)) != len(f):
        raise ValueError('Duplicate faces')
    graph = coo_matrix((np.ones(len(unique)), (unique[:, 0], unique[:, 1])),
                       shape=(len(v), len(v))).tocsr()
    n, labels = connected_components(graph, directed=False)
    signature = []
    for k in range(n):
        nv = np.count_nonzero(labels == k)
        ne = np.count_nonzero(labels[unique[:, 0]] == k)
        nf = np.count_nonzero(labels[f[:, 0]] == k)
        signature.append(int(nv - ne + nf))
    return sorted(signature)


def volume(v, f):
    # Translate before summing to limit cancellation for small remote parts.
    t = v[f] - v.mean(axis=0)
    return float(np.einsum('ij,ij->i', t[:, 0],
                           np.cross(t[:, 1], t[:, 2])).sum() / 6)


def repair(v, f, max_distance):
    original_signature = topology(v, f)
    original_volume = volume(v, f)
    log = []
    for _ in range(50):
        bad = np.flatnonzero(areas(v, f) < 1.05e-10)
        if not len(bad):
            return v, f, log
        options = []
        for face in f[bad]:
            for a, b in [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])]:
                options.append((float(np.linalg.norm(v[a] - v[b])), int(a), int(b)))
        accepted = False
        for distance, a, b in sorted(set(options)):
            if distance > max_distance:
                continue
            for weight in [.5, 0., 1.]:
                vv = v.copy()
                vv[a] = (1-weight)*v[a] + weight*v[b]
                ff = f.copy()
                ff[ff == b] = a
                keep = ((ff[:, 0] != ff[:, 1]) & (ff[:, 1] != ff[:, 2]) &
                        (ff[:, 2] != ff[:, 0]))
                ff = ff[keep]
                # Reject inversions of surviving triangles.
                old_normals = np.cross(v[f[keep, 1]]-v[f[keep, 0]],
                                       v[f[keep, 2]]-v[f[keep, 0]])
                new_normals = np.cross(vv[ff[:, 1]]-vv[ff[:, 0]],
                                       vv[ff[:, 2]]-vv[ff[:, 0]])
                if np.any(np.einsum('ij,ij->i', old_normals, new_normals) <= 0):
                    continue
                vv, ff = compact(vv, ff)
                if np.count_nonzero(areas(vv, ff) < 1.05e-10) >= len(bad):
                    continue
                try:
                    if topology(vv, ff) != original_signature:
                        continue
                except ValueError:
                    continue
                if abs(volume(vv, ff)/original_volume - 1) > .001:
                    continue
                log.append(dict(edge_length_cm=distance, faces_removed=int(len(f)-len(ff))))
                v, f = vv, ff
                accepted = True
                break
            if accepted:
                break
        if not accepted:
            raise ValueError('No safe local collapse found for small facets')
    raise ValueError('Repair did not converge')


def samples(v, f, seed):
    rng = np.random.default_rng(seed)
    vi = rng.choice(len(v), min(1500, len(v)), replace=False)
    fi = rng.choice(len(f), min(1500, len(f)), replace=False)
    return np.vstack([v[vi], v[f[fi]].mean(axis=1)])


def sampled_distance(v, f, vv, ff):
    before = trimesh.Trimesh(v, f, process=False)
    after = trimesh.Trimesh(vv, ff, process=False)
    distances = []
    for mesh, points in [(before, samples(vv, ff, 1)),
                         (after, samples(v, f, 2))]:
        for start in range(0, len(points), 256):
            distances.extend(trimesh.proximity.closest_point(mesh, points[start:start+256])[1])
    return float(np.max(distances))


def simplify(v, f, tolerance, volume_limit):
    signature = topology(v, f)
    base_volume = volume(v, f)
    base_bounds = np.array([v.min(axis=0), v.max(axis=0)])
    # Try a 50% target, then a gentler target, then coplanar simplification.
    for method, kwargs in [('half', dict(target_reduction=.5, agg=5)),
                            ('quarter', dict(target_reduction=.25, agg=3)),
                            ('lossless', dict(lossless=True))]:
        vv, ff = fast_simplification.simplify(v, f, **kwargs)
        vv, ff = compact(vv, ff)
        if len(ff) >= len(f) or np.any(areas(vv, ff) < 1.05e-10):
            continue
        delta_volume = abs(volume(vv, ff)/base_volume - 1)
        delta_bounds = float(np.max(np.abs(np.array([vv.min(axis=0), vv.max(axis=0)])-base_bounds)))
        if delta_volume > volume_limit or delta_bounds > tolerance:
            continue
        try:
            if topology(vv, ff) != signature:
                continue
        except ValueError:
            continue
        distance = sampled_distance(v, f, vv, ff)
        if distance <= tolerance:
            return vv, ff, method, delta_volume, distance, delta_bounds
    return v, f, 'unchanged', 0., 0., 0.


def write_mesh(path, text, v, f):
    # Preserve volume name, material overrides, placement, and other metadata.
    prefix = re.search(r'^(\S+)\.Vertices ', text, re.M).group(1)
    result = []
    for line in text.splitlines():
        t = line.split()
        if t and t[0].endswith(('.Vertex', '.Facet', '.Facets')):
            continue
        if t and t[0].endswith('.Shape') and t[1] == 'TESSELLATED':
            line = f'{t[0]} TESSELLATED {len(v)} {len(f)}'
        elif t and t[0].endswith('.Vertices'):
            result.append(f'{prefix}.Vertices {len(v)}')
            result.extend(f'{prefix}.Vertex ' + ' '.join(format(float(x), '.17g') for x in p) for p in v)
            result.append(f'{prefix}.Facets {len(f)}')
            result.extend(f'{prefix}.Facet {a} {b} {c}' for a, b, c in f)
            continue
        result.append(line)
    path.write_text('\n'.join(result)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--setup', type=Path, default=Path('output/cosi_test.geo.setup'))
    parser.add_argument('--distance-cm', type=float, default=.005)
    parser.add_argument('--volume-fraction', type=float, default=.001)
    args = parser.parse_args()
    parent = args.setup.parent
    setup = args.setup.read_text()
    source_name = 'cosi_test_geo'
    variants = {mode: parent/f'cosi_test_{mode}_geo' for mode in ['repaired', 'optimized']}
    for folder in variants.values():
        folder.mkdir(exist_ok=False)
    report = []
    for index, path in enumerate(sorted((parent/source_name).glob('*.geo'))):
        text, v, f = read_mesh(path)
        original_faces = len(f)
        original_volume = volume(v, f)
        if original_volume <= 0:
            raise ValueError(f'{path}: nonpositive volume')
        bad = int(np.count_nonzero(areas(v, f) < 1e-10))
        v, f, repairs = repair(v, f, args.distance_cm)
        write_mesh(variants['repaired']/path.name, text, v, f)
        vv, ff, method, dv, distance, bounds = simplify(v, f, args.distance_cm, args.volume_fraction)
        write_mesh(variants['optimized']/path.name, text, vv, ff)
        report.append(dict(solid=path.stem, original_faces=original_faces,
                           original_problem_facets=bad, edge_collapses=len(repairs),
                           repaired_faces=len(f), optimized_faces=len(ff), method=method,
                           repair_volume_fraction=abs(volume(v, f)/original_volume-1),
                           max_repair_edge_cm=max([r['edge_length_cm'] for r in repairs], default=0),
                           simplification_volume_fraction=dv, sampled_distance_cm=distance,
                           bounds_change_cm=bounds))
        if repairs or index % 20 == 0:
            print(f'{index+1}: {path.stem}: {original_faces} -> {len(f)} -> {len(ff)} ({method}), repairs={len(repairs)}', flush=True)
    for mode, folder in variants.items():
        (parent/f'cosi_test_{mode}.geo.setup').write_text(
            f'// {mode} variant; original assembly preserved.\n' +
            setup.replace(source_name+'/', folder.name+'/'))
    with (parent/'cosi_optimization_report.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(report[0]))
        writer.writeheader()
        writer.writerows(report)
    totals = {key: sum(r[key] for r in report) for key in
              ['original_faces', 'repaired_faces', 'optimized_faces', 'original_problem_facets', 'edge_collapses']}
    totals.update(solids=len(report), distance_limit_cm=args.distance_cm,
                  volume_fraction_limit=args.volume_fraction,
                  maximum_sampled_distance_cm=max(r['sampled_distance_cm'] for r in report),
                  maximum_simplification_volume_fraction=max(r['simplification_volume_fraction'] for r in report))
    (parent/'cosi_optimization_summary.json').write_text(json.dumps(totals, indent=2)+'\n')
    print(json.dumps(totals, indent=2), flush=True)


if __name__ == '__main__':
    main()
