mod cbs;

use std::fs;
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
    timeout: Option<u64>, // TODO: implement

    #[arg(
        short = 'k',
        long,
        help = "Consider only the first k agents of the scenario."
    )]
    num_agents: Option<usize>, // TODO: implement

    #[arg(long, default_value = "false")]
    disable_diagonal_subsolver: bool,

    #[arg(long, default_value = "false")]
    disable_bypassing_conflicts: bool,

    #[arg(long, default_value = "false")]
    disable_prioritising_conflicts: bool,
}

fn main() {
    env_logger::init();
    let args = Args::parse();
    let cbs_instance = CBSInstance::from_files(&args.map_file, &args.agents_file)
        .expect("should be valid scenario files");
    let optimisation_config = Some(cbs::CBSOptimisationConfig::new(
        !args.disable_prioritising_conflicts,
        !args.disable_bypassing_conflicts,
        !args.disable_diagonal_subsolver,
    ));
    let mut cbs = CBS::new(cbs_instance, optimisation_config);
    match cbs.solve() {
        Ok(paths) => {
            if let Some(paths_file) = args.paths_file {
                fs::write(paths_file, paths_to_string(&paths)).expect("should write paths file");
            } else {
                println!("{}", paths_to_string(&paths));
            }
            if let Some(metrics_file) = args.metrics_file {
                fs::write(
                    metrics_file,
                    format!("#high-level generated\n{}", cbs.high_level_generated),
                )
                .expect("should write metrics file");
            }
        }
        Err(e) => panic!("CBS Error: {:?}", e),
    }
}
