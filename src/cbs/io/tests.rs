use super::*;
use rstest::rstest;
use std::io::Read;

#[rstest]
#[case::clean_24x24("tests/testdata/maps/clean_24x24.map", 24, 24, vec![])]
#[case::clean_24x24_crlf("tests/testdata/maps/clean_24x24_crlf.map", 24, 24, vec![])]
#[case::clean_24x24_crlf("tests/testdata/maps/5x3_with_conflict.map", 5, 3, vec![
    LocationTime { location: (3, 1), time: -1 },
])]
fn test_map_loading_saving(
    #[case] map_file_path: &str,
    #[case] height: i32,
    #[case] width: i32,
    #[case] obstacles: Vec<LocationTime>,
) {
    let mut map_file = File::open(map_file_path).unwrap();
    let mut map_file_content = String::new();
    map_file.read_to_string(&mut map_file_content).unwrap();
    let map = Grid::try_from(map_file_content).unwrap();
    assert_eq!(map.height, height);
    assert_eq!(map.width, width);
    assert_eq!(map.obstacles, obstacles.clone().into_iter().collect());

    let map_file_content: String = map.try_into().unwrap();
    let map = Grid::try_from(map_file_content).unwrap();
    assert_eq!(map.height, height);
    assert_eq!(map.width, width);
    assert_eq!(map.obstacles, obstacles.into_iter().collect());
}

#[rstest]
#[case::simple_24x24_12_agents("tests/testdata/scenarios/24x24_12_agents.scen", vec![
    Agent { id: "0".to_string(), start: (11, 0), goal: (11, 11) },
    Agent { id: "1".to_string(), start: (0, 11), goal: (11, 22) },
    Agent { id: "2".to_string(), start: (10, 1), goal: (23, 12) },
    Agent { id: "3".to_string(), start: (9, 2), goal: (23, 13) },
    Agent { id: "4".to_string(), start: (8, 3), goal: (23, 14) },
    Agent { id: "5".to_string(), start: (7, 4), goal: (23, 15) },
    Agent { id: "6".to_string(), start: (6, 5), goal: (23, 16) },
    Agent { id: "7".to_string(), start: (5, 6), goal: (23, 17) },
    Agent { id: "8".to_string(), start: (4, 7), goal: (23, 18) },
    Agent { id: "9".to_string(), start: (3, 8), goal: (23, 19) },
    Agent { id: "10".to_string(), start: (2, 9), goal: (23, 20) },
    Agent { id: "11".to_string(), start: (1, 10), goal: (23, 21) },
])]
fn test_load_scenario_file(#[case] scen_file_path: &str, #[case] exp_agents: Vec<Agent>) {
    let agents = load_scenario_file(scen_file_path).unwrap();
    assert_eq!(agents, exp_agents);
}
