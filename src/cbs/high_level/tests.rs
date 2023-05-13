use crate::cbs::optimisations::conflict_prioritisation::pick_conflict;

use super::*;

#[test]
#[ignore = "non deterministic"]
fn test_high_level() {
    let agents = vec![
        Agent {
            id: "a".to_string(),
            start: (0, 0),
            goal: (9, 9),
        },
        Agent {
            id: "b".to_string(),
            start: (0, 1),
            goal: (9, 8),
        },
    ];
    let constraints = vec![
        Box::new(Constraint {
            agent: &agents[0],
            time: 1,
            location: (1, 0),
        }),
        Box::new(Constraint {
            agent: &agents[1],
            time: 1,
            location: (1, 1),
        }),
        Box::new(Constraint {
            agent: &agents[1],
            time: 1,
            location: (0, 2),
        }),
    ];
    let precomputed_paths = HashMap::<&Agent, Vec<(i32, i32)>>::new();
    let grid = Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]), (9, 9));
    let ctn = ConflictTreeNode::new(
        agents.iter().collect(),
        constraints,
        precomputed_paths,
        &grid,
        Some(pick_conflict),
        None,
        None,
    );
    assert_eq!(ctn.conflicts.len(), 13);
    match ctn.expand() {
        Some(expanded) => {
            assert_eq!(expanded.len(), 2);
            assert_eq!(expanded[0].constraints.len(), 4);
            assert_eq!(expanded[1].constraints.len(), 4);
            assert_eq!(expanded[0].constraints[3].time, 1);
            assert_eq!(expanded[0].conflicts.len(), 1);
        }
        None => panic!("No expanded nodes"),
    }
}
