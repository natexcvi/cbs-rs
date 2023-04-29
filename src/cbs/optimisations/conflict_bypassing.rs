use crate::cbs::high_level::{Agent, Conflict, ConflictTreeNode, Path};

pub fn bypass_conflict<'a>(
    parent: &ConflictTreeNode<'a>,
    children: Vec<Box<ConflictTreeNode<'a>>>,
) -> Option<Vec<Box<ConflictTreeNode<'a>>>> {
    for child in children.iter() {
        for (agent, path) in child.paths.iter() {
            if path.len() > parent.paths[agent].len() {
                continue;
            }
            let mut new_parent = parent.clone();
            new_parent.paths.insert(agent, path.clone());
            new_parent.compute_conflicts();
            if new_parent.conflicts.len() >= parent.conflicts.len() {
                continue;
            }
            return Some(vec![Box::new(new_parent)]);
        }
    }
    Some(children)
}
