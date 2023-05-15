use super::*;
use rstest::rstest;
use std::hash::Hash;

#[derive(Debug, Clone, PartialEq)]
struct TestNode {
    id: String,
    score: f64,
    h: f64,
    expand: Vec<Box<TestNode>>,
}

impl Hash for TestNode {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Eq for TestNode {}

impl AStarNode<'_> for TestNode {
    fn g(&self) -> f64 {
        self.score
    }

    fn h(&self) -> f64 {
        self.h
    }

    fn is_goal(&self) -> bool {
        self.h == 0.0
    }

    fn tie_breaker(&self, other: &Self) -> std::cmp::Ordering {
        std::cmp::Ordering::Equal
    }

    fn expand(&self) -> Option<Vec<Box<Self>>> {
        Some(self.expand.iter().map(|x| x.clone()).collect())
    }

    fn id(&self) -> String {
        self.id.clone()
    }
}

#[test]
fn test_a_star() {
    let mut a = TestNode {
        id: "a".to_string(),
        score: 0.0,
        h: 1.0,
        expand: Vec::new(),
    };
    let mut b = TestNode {
        id: "b".to_string(),
        score: 1.0,
        h: 1.0,
        expand: Vec::new(),
    };
    let mut c = TestNode {
        id: "c".to_string(),
        score: 2.0,
        h: 1.0,
        expand: Vec::new(),
    };
    let d = TestNode {
        id: "d".to_string(),
        score: 3.0,
        h: 0.0,
        expand: Vec::new(),
    };
    let e = TestNode {
        id: "e".to_string(),
        score: 4.0,
        h: 0.0,
        expand: Vec::new(),
    };
    c.expand.push(Box::new(e));
    c.expand.push(Box::new(d));
    b.expand.push(Box::new(c));
    a.expand.push(Box::new(b));
    let result = a_star(a).unwrap();
    assert_eq!(result.path[result.path.len() - 1].id, "d");
    assert_eq!(result.path[result.path.len() - 2].id, "c");
    assert_eq!(result.path[result.path.len() - 3].id, "b");
    assert_eq!(result.path[result.path.len() - 4].id, "a");
}

#[rstest]
#[case(
    vec![].into_iter().collect::<HashSet<String>>(),
    &|path: &mut Vec<String>, cur: &String, parent: &Option<String>| {
        path.push(cur.clone());
        true
    },
    &|path: &mut Vec<String>, cur: &String, _: &Option<String>| {
        if cur != "g" {
            path.pop();
        }
    },
    "a".to_string(),
    None,
    &|cur: &String| -> Vec<String> {
        if cur == "a" {
            return vec!["b", "c", "d"].into_iter().map(|s| s.to_string()).collect();
        } else if cur == "d" {
            return vec!["e", "g"].into_iter().map(|s| s.to_string()).collect();
        }
        return vec![];
    },
    &|cur: &String| cur == "g",
    vec!["a", "d", "g"].into_iter().map(|s| s.to_string()).collect(),
)]
#[case(
    vec![].into_iter().collect::<HashSet<String>>(),
    &|path: &mut Vec<String>, cur: &String, parent: &Option<String>| {
        path.push(cur.clone());
        true
    },
    &|path: &mut Vec<String>, cur: &String, _: &Option<String>| {
        if cur != "g" {
            path.pop();
        }
    },
    "a".to_string(),
    None,
    &|cur: &String| -> Vec<String> {
        if cur == "a" {
            return vec!["b", "c", "d"].into_iter().map(|s| s.to_string()).collect();
        } else if cur == "d" {
            return vec!["e", "g"].into_iter().map(|s| s.to_string()).collect();
        } else if cur == "c" {
            return vec!["g"].into_iter().map(|s| s.to_string()).collect();
        }
        return vec![];
    },
    &|cur: &String| cur == "g",
    vec!["a", "c", "g"].into_iter().map(|s| s.to_string()).collect(),
)]
fn test_dfs(
    #[case] init_visited: HashSet<String>,
    #[case] processor: &dyn Fn(&mut Vec<String>, &String, &Option<String>) -> bool,
    #[case] on_backtrack: &dyn Fn(&mut Vec<String>, &String, &Option<String>),
    #[case] cur: String,
    #[case] parent: Option<String>,
    #[case] neighbours: &dyn Fn(&String) -> Vec<String>,
    #[case] is_goal: &dyn Fn(&String) -> bool,
    #[case] expected: Vec<String>,
) {
    let mut visited = HashSet::new();
    visited.extend(init_visited);
    let mut path = vec![];
    dfs(
        &mut visited,
        &mut path,
        processor,
        on_backtrack,
        cur,
        parent,
        neighbours,
        is_goal,
    );
    assert_eq!(path, expected);
}
