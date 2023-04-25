use super::*;

#[derive(Debug, Clone)]
struct TestNode {
    id: String,
    score: f64,
    h: f64,
    expand: Vec<Box<TestNode>>,
}

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

    fn expand(&self) -> Option<Vec<Box<Self>>> {
        Some(self.expand.iter().map(|x| x.clone()).collect())
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
