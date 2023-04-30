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

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::bypass_conflict;
    use crate::cbs::{
        high_level::{Agent, Conflict, ConflictTreeNode, Path, VertexConflict},
        low_level::Grid,
    };
    use rstest::rstest;

    #[rstest]
    #[case::simple(
        vec![
            vec![(0, 0), (1, 0), (2, 0)].into(),
            vec![(1, 0), (1, 0), (1, 1)].into()
        ],
        0,
        vec![
            vec![
                vec![(0, 0), (1, 0), (2, 0)].into(),
                vec![(1, 0), (1, 1), (1, 1)].into()
            ],
            vec![
                vec![(0, 0), (0, 0), (1, 0), (2, 0)].into(),
                vec![(1, 0), (1, 0), (1, 1)].into()
            ],
        ],
        true,
    )]
    #[case::no_bypass(
        vec![
            vec![(0, 0), (1, 0)].into(),
            vec![(1, 0), (0, 0)].into()
        ],
        0,
        vec![
            vec![
                vec![(0, 0), (1, 0)].into(),
                vec![(1, 0), (1, 1), (0, 1), (0, 0)].into()
            ],
            vec![
                vec![(0, 0), (0, 1), (0, 0), (1, 0)].into(),
                vec![(1, 0), (0, 0)].into()
            ],
        ],
        false,
    )]
    fn test_bypass_conflict(
        #[case] parent_paths: Vec<Path>,
        #[case] conflict_idx: usize,
        #[case] children_paths: Vec<Vec<Path>>,
        #[case] expect_bypass: bool,
    ) {
        let mut agents = Vec::new();
        let parent_paths: Vec<(usize, Path)> = parent_paths
            .into_iter()
            .enumerate()
            .map(|(i, path)| (i, path))
            .collect();
        for (i, path) in parent_paths.iter() {
            agents.push(Agent {
                id: i.to_string(),
                start: path[0],
                goal: path[path.len() - 1],
            });
        }
        let scenario = Grid::new(10, 5, vec![], (0, 0));
        let parent = ConflictTreeNode::new(
            agents.iter().collect(),
            vec![],
            parent_paths
                .into_iter()
                .enumerate()
                .map(|(i, path)| (&agents[i], path.1))
                .collect(),
            &scenario,
            None,
            None,
        );
        let conflict = parent.conflicts[conflict_idx].clone();
        let children: Vec<ConflictTreeNode> = children_paths
            .into_iter()
            .map(|paths| {
                ConflictTreeNode::new(
                    agents.iter().collect(),
                    vec![],
                    paths
                        .into_iter()
                        .enumerate()
                        .map(|(i, path)| (&agents[i], path))
                        .collect(),
                    &scenario,
                    None,
                    None,
                )
            })
            .collect();
        let children: Vec<Box<ConflictTreeNode>> = children.into_iter().map(Box::new).collect();
        let result = bypass_conflict(&parent, &conflict, children.clone());
        assert!(result.is_some());
        let result = result.unwrap();
        if !expect_bypass {
            assert_eq!(result.len(), children.len());
        } else {
            assert_eq!(result.len(), 1);
        }
    }
}
