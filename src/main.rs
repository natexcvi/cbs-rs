mod cbs;

use std::fs;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::{thread::JoinHandle, time::Duration};

use cbs::io::paths_to_string;
use cbs::{CBSInstance, CBS};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    map_file: String,

    #[arg(short, long)]
    agents_file: String,

    #[arg(
        short,
        long,
        help = "Output file for paths. If not specified, will print to stdout"
    )]
    paths_file: Option<String>,

    #[arg(long)]
    metrics_file: Option<String>,

    #[arg(
        short,
        long,
        help = "Stop if a solution is not found withing this number of seconds."
    )]
    timeout: Option<u64>,

    #[arg(
        short = 'k',
        long,
        help = "Consider only the first k agents of the scenario."
    )]
    num_agents: Option<usize>,

    #[arg(long, default_value = "false")]
    disable_diagonal_subsolver: bool,

    #[arg(long, default_value = "false")]
    disable_bypassing_conflicts: bool,

    #[arg(long, default_value = "false")]
    disable_prioritising_conflicts: bool,

    #[arg(
        long = "disable-cat",
        default_value = "false",
        help = "Disable the conflict avoidance table"
    )]
    disable_conflict_avoidance_table: bool,
}

fn main() {
    env_logger::init();
    let args = Args::parse();
    let cbs_instance = CBSInstance::from_files(&args.map_file, &args.agents_file, args.num_agents)
        .expect("should be valid scenario files");
    let optimisation_config = Some(cbs::CBSOptimisationConfig::new(
        !args.disable_prioritising_conflicts,
        !args.disable_bypassing_conflicts,
        !args.disable_diagonal_subsolver,
        !args.disable_conflict_avoidance_table,
    ));
    let mut cbs = CBS::new(cbs_instance, optimisation_config);
    let is_solving = Arc::new(AtomicBool::new(true));
    if let Some(timeout) = args.timeout {
        let timeout = Duration::from_secs(timeout);
        let is_solving = is_solving.clone();
        start_timeout_thread(timeout, is_solving);
    }
    let solution = cbs.solve();
    is_solving.store(false, Ordering::SeqCst);
    match solution {
        Ok(paths) => {
            write_paths(&args.paths_file, paths_to_string(&paths));
            if let Some(metrics_file) = args.metrics_file {
                write_metrics(metrics_file, cbs);
            }
        }
        Err(e) => panic!("CBS Error: {:?}", e),
    }
}

fn write_paths(paths_file: &Option<String>, paths_string: String) {
    if let Some(paths_file) = paths_file {
        fs::write(paths_file, paths_string).expect("should write paths file");
    } else {
        println!("{}", paths_string);
    }
}

fn write_metrics(metrics_file: String, cbs: CBS) {
    fs::write(
        metrics_file,
        format!("#high-level generated\n{}", cbs.high_level_generated),
    )
    .expect("should write metrics file");
}

fn start_timeout_thread(timeout: Duration, is_solving: Arc<AtomicBool>) -> JoinHandle<()> {
    std::thread::spawn(move || {
        std::thread::sleep(timeout);
        if is_solving.load(Ordering::SeqCst) {
            log::error!("Timed out");
            std::process::exit(1);
        }
    })
}
