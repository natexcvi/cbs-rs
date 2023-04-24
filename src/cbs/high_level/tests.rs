use super::*;

#[test]
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
            time: 0,
            location: (1, 0),
        }),
        Box::new(Constraint {
            agent: &agents[1],
            time: 0,
            location: (8, 0),
        }),
    ];
    let precomputed_paths = HashMap::<&Agent, Vec<(i32, i32)>>::new();
    let grid = Grid {
        width: 10,
        height: 10,
        obstacles: vec![],
        goal: (9, 9),
    };
    let ctn = ConflictTreeNode::new(
        agents.iter().collect(),
        constraints,
        precomputed_paths,
        &grid,
    );
    assert_eq!(ctn.conflicts.len(), 1);
    match ctn.expand() {
        Some(expanded) => {
            assert_eq!(expanded.len(), 2);
            assert_eq!(expanded[0].constraints.len(), 3);
            assert_eq!(expanded[1].constraints.len(), 3);
            assert_eq!(expanded[0].constraints[2].time, 17);
            assert_eq!(expanded[0].conflicts.len(), 0);
        }
        None => panic!("No expanded nodes"),
    }
}

#[test]
fn test_cbs() {
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
    let grid = Grid {
        width: 10,
        height: 10,
        obstacles: vec![],
        goal: (9, 9),
    };
    let cbs = CBS::new(&grid, agents.iter().collect());
    match cbs.solve() {
        Ok(paths) => {
            assert_eq!(paths.len(), 2);
            assert_eq!(paths[&agents[0]].len(), 18);
            assert_eq!(paths[&agents[1]].len(), 18);
        }
        Err(e) => panic!("Error: {:?}", e),
    }
}