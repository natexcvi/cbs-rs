use super::*;
use rstest::rstest;

#[rstest]
#[case::simple(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
        two_direction_subsolver: true,
    }),
    vec![
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
    ], Grid::new(
        10,
        10,
        Grid::to_conditional_obstacles(Vec::new()),
        (9, 9),
    ),
    vec![19, 17],
)]
#[case::opposite_corners(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
        two_direction_subsolver: true,
    }),
    vec![
        Agent {
            id: "a".to_string(),
            start: (9, 9),
            goal: (0, 0),
        },
        Agent {
            id: "b".to_string(),
            start: (0, 0),
            goal: (9, 9),
        },
    ], Grid::new(
        10,
        10,
        Grid::to_conditional_obstacles(Vec::new()),
        (9, 9),
    ),
    vec![19, 19],
)]
#[case::crowded(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
        two_direction_subsolver: true,
    }),
    vec![
        Agent {
            id: "a".to_string(),
            start: (0, 0),
            goal: (1, 1),
        },
        Agent {
            id: "b".to_string(),
            start: (1, 0),
            goal: (0, 0),
        },
        Agent {
            id: "c".to_string(),
            start: (0, 1),
            goal: (1, 0),
        },
    ], Grid::new(
        2,
        2,
        Grid::to_conditional_obstacles(Vec::new()),
        (0, 0),
    ),
    vec![3, 2, 3],
)] // TODO: confirm this case
#[case::crowded_with_obstacles(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
        two_direction_subsolver: true,
    }),
    vec![
        Agent {
            id: "a".to_string(),
            start: (0, 0),
            goal: (2, 2),
        },
        Agent {
            id: "b".to_string(),
            start: (1, 0),
            goal: (0, 0),
        },
        Agent {
            id: "c".to_string(),
            start: (0, 1),
            goal: (1, 0),
        },
    ], Grid::new(
        3,
        3,
        Grid::to_conditional_obstacles(vec![low_level::LocationTime { location: (1,1), time: 2 }]),
        (0, 0),
    ),
    vec![5, 2, 3],
)]
#[case::must_wait(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
        two_direction_subsolver: true,
    }),
    vec![
        Agent {
            id: "a".to_string(),
            start: (0, 1),
            goal: (2, 1),
        },
        Agent {
            id: "b".to_string(),
            start: (1, 0),
            goal: (1, 2),
        },
    ], Grid::new(
        3,
        3,
        Grid::to_conditional_obstacles(vec![low_level::LocationTime { location: (2,0), time: -1 }, low_level::LocationTime { location: (0,2), time: -1 }]),
        (0, 0),
    ),
    vec![3, 4],
)]
fn test_cbs(
    #[case] optimisation_config: Option<CBSOptimisationConfig>,
    #[case] agents: Vec<Agent>,
    #[case] grid: Grid,
    #[case] exp_path_lengths: Vec<usize>,
) {
    let mut cbs = CBS::new(
        CBSInstance {
            map: grid,
            agents: agents.clone(),
        },
        optimisation_config,
    );
    match cbs.solve() {
        Ok(paths) => {
            assert_eq!(paths.len(), exp_path_lengths.len());
            let exp_path_lengths = exp_path_lengths
                .into_iter()
                .enumerate()
                .map(|(i, l)| (&agents[i], l))
                .collect::<HashMap<_, _>>();
            assert_eq!(
                paths
                    .into_iter()
                    .map(|(a, p)| (a, p.len()))
                    .collect::<HashMap<&Agent, usize>>(),
                exp_path_lengths
            );
        }
        Err(e) => panic!("Unexpected error: {:?}", e),
    }
}

#[rstest]
#[case::simple(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
        two_direction_subsolver: false,
    }),
    "tests/testdata/maps/empty-16-16.map",
    "tests/testdata/scenarios/empty-16-16-even-1.scen",
    vec![6, 20, 7, 23, 15]
)]
#[ignore = "investigate bad performance"]
#[case::maze_128x128(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
        two_direction_subsolver: false,
    }),
    "tests/testdata/maps/maze-128-128-10.map",
    "tests/testdata/scenarios/maze-128-128-10-even-1.scen",
    vec![305, 364, 134]
)]
#[case::diagonal_10(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
        two_direction_subsolver: false,
    }),
    "tests/testdata/maps/test_10.map",
    "tests/testdata/scenarios/test_10.scen",
    vec![12, 23, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34]
)] // TODO: confirm this case
fn test_cbs_from_files(
    #[case] optimisation_config: Option<CBSOptimisationConfig>,
    #[case] map_file: &str,
    #[case] scenario_file: &str,
    #[case] exp_path_lengths: Vec<usize>,
) {
    let cbs_instance =
        CBSInstance::from_files(map_file, scenario_file).expect("should be valid scenario files");
    let mut cbs = CBS::new(cbs_instance, optimisation_config);
    match cbs.solve() {
        Ok(paths) => {
            assert_eq!(paths.len(), exp_path_lengths.len());
            let mut paths = paths.iter().collect::<Vec<_>>();
            paths.sort_by_key(|(agent, _)| {
                agent
                    .id
                    .parse::<usize>()
                    .expect("agent id should be numeric")
            });
            let exp_path_lengths = paths
                .iter()
                .map(|(agent, _)| {
                    exp_path_lengths[agent
                        .id
                        .parse::<usize>()
                        .expect("agent id should be numeric")]
                })
                .collect::<Vec<usize>>();
            assert_eq!(
                paths
                    .iter()
                    .map(|(_, path)| path.len())
                    .collect::<Vec<usize>>(),
                exp_path_lengths
            );
        }
        Err(e) => panic!("Error: {:?}", e),
    }
}
