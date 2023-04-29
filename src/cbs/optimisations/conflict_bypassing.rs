use crate::cbs::high_level::{Conflict, ConflictTreeNode};

pub fn bypass_conflict<'a>(
    parent: &ConflictTreeNode<'a>,
    conflict: &Conflict<'a>,
    children: Vec<Box<ConflictTreeNode<'a>>>,
) -> Option<Vec<Box<ConflictTreeNode<'a>>>> {
    let (agent1, agent2) = match conflict {
        Conflict::Vertex(conflict) => (conflict.agent1, conflict.agent2),
        Conflict::Edge(conflict) => (conflict.agent1, conflict.agent2),
    };
    for child in children.iter() {
        for agent in vec![agent1, agent2] {
            let path = child.paths[agent].clone();
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
