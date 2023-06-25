use super::*;
use rstest::rstest;

#[rstest]
#[case(
    vec![('a', 'b'), ('a', 'c'), ('b', 'c'), ('b', 'd'), ('c', 'd')],
    2,
    Some(2),
)]
#[case(
    vec![('a', 'b'), ('a', 'c'), ('b', 'c'), ('b', 'd'), ('c', 'd'), ('c', 'e')],
    4,
    Some(2),
)]
#[case(
    vec![('a', 'b'), ('a', 'c'), ('b', 'c'), ('b', 'd'), ('c', 'd')],
    1,
    None,
)]
#[case(
    vec![('a', 'b'), ('a', 'c'), ('b', 'c'), ('b', 'd'), ('c', 'd')],
    6,
    Some(2),
)]
#[case::k_is_zero_no_cover(
    vec![('a', 'b'), ('a', 'c'), ('b', 'c'), ('b', 'd'), ('c', 'd')],
    0,
    None,
)]
#[case::empty_graph_empty_cover(
    vec![],
    3,
    Some(0),
)]
fn test_min_vertex_cover(
    #[case] edges: Vec<(char, char)>,
    #[case] k: usize,
    #[case] expected: Option<usize>,
) {
    let mut graph = MVCGraph::<char>::new();
    let mut vertices = HashSet::<char>::new();
    for edge in edges.iter() {
        vertices.insert(edge.0);
        vertices.insert(edge.1);
        graph.add_edge(Rc::new(edge.0), Rc::new(edge.1));
    }
    let cover = min_vertex_cover(&graph, k);
    match cover {
        Some(cover) => {
            assert!(
                expected.is_some(),
                "{}",
                format!("expected cover to be None but it was {:?}", cover),
            );
            let mut cover_set = HashSet::<char>::new();
            for vertex in cover.iter() {
                cover_set.insert(**vertex);
            }
            for edge in edges.iter() {
                assert!(
                    cover_set.contains(&edge.0) || cover_set.contains(&edge.1),
                    "{}",
                    format!("vertex cover {:?} should cover edge {:?}", cover_set, edge)
                );
            }
            assert_eq!(
                expected,
                Some(cover.len()),
                "{}",
                format!(
                    "vertex cover {:?} should have size {}",
                    cover,
                    expected.unwrap()
                )
            );
        }
        None => assert_eq!(expected, None),
    }
}
