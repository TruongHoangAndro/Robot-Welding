import { useMemo, useState, type JSX } from 'react';
import { ROLE_COLOUR, ROLE_LABEL, effectiveRoles, relPaths, setRole, type Assignable, type Assignment, type TreeNode } from '../lib/roles';

/**
 * Component tree of a STEP assembly with one role per node (CP-60). A role set on a node applies to its children
 * unless a child has its own (shown greyed as "inherited"); "inherit" clears the node's own role.
 */
export function RoleTree(props: {
  tree: TreeNode[];
  assignments: Assignment[];
  roles: Assignable[];
  disabled: boolean;
  selected: number | null;
  onSelect: (index: number) => void;
  onChange: (a: Assignment[]) => void;
}) {
  const { tree, assignments, roles, disabled, selected } = props;
  const rel = useMemo(() => relPaths(tree), [tree]);
  const eff = useMemo(() => effectiveRoles(tree, assignments), [tree, assignments]);
  const own = useMemo(() => new Map(assignments.map((a) => [a.path, a.role])), [assignments]);
  const kids = useMemo(() => {
    const m = new Map<number, number[]>();
    for (const n of tree) if (n.parent !== null) m.set(n.parent, [...(m.get(n.parent) ?? []), n.index]);
    return m;
  }, [tree]);
  const [open, setOpen] = useState<Set<number>>(() => new Set(tree.filter((n) => n.parent === null).map((n) => n.index)));
  const toggle = (i: number) => setOpen((s) => (s.has(i) ? new Set([...s].filter((k) => k !== i)) : new Set(s).add(i)));

  const row = (i: number, depth: number): JSX.Element[] => {
    const n = tree[i];
    const children = kids.get(i) ?? [];
    const out: JSX.Element[] = [];
    if (n.parent !== null) {
      const mine = own.get(rel[i]);
      out.push(
        <div key={i} className={`tree-row${selected === i ? ' active' : ''}`} style={{ paddingLeft: 6 + depth * 14 }} onClick={() => props.onSelect(i)}>
          {children.length ? (
            <button className="twisty" onClick={(e) => (e.stopPropagation(), toggle(i))}>
              {open.has(i) ? '▾' : '▸'}
            </button>
          ) : (
            <span className="twisty" />
          )}
          <span className="chip" style={{ background: ROLE_COLOUR[eff[i]] }} title={ROLE_LABEL[eff[i]]} />
          <span className={`name${eff[i] === 'exclude' ? ' excluded' : ''}`} title={rel[i]}>
            {n.instance_name || n.product_name}
          </span>
          <select
            value={mine ?? 'inherit'}
            disabled={disabled}
            onClick={(e) => e.stopPropagation()}
            onChange={(e) => props.onChange(setRole(props.assignments, rel[i], e.target.value as Assignable | 'inherit'))}
          >
            <option value="inherit">{`inherit (${ROLE_LABEL[n.parent !== null ? eff[n.parent] : 'static']})`}</option>
            {roles.map((r) => (
              <option key={r} value={r}>
                {ROLE_LABEL[r]}
              </option>
            ))}
          </select>
        </div>,
      );
    }
    if (n.parent === null || open.has(i)) for (const c of children) out.push(...row(c, n.parent === null ? 0 : depth + 1));
    return out;
  };

  return <div className="tree">{tree.filter((n) => n.parent === null).flatMap((n) => row(n.index, 0))}</div>;
}
