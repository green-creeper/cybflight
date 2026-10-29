mod app;
mod backend;
mod ui;

use clap::Parser;

use crate::app::CybflightGsApp;

#[derive(Parser, Debug)]
#[command(
    name = "cybflight-gs",
    about = "Cybflight Ground Station & Betaflight-style Configurator"
)]
struct Args {
    /// Initial vehicle YAML stem to load (default: goku_f745).
    #[arg(short, long, default_value = "goku_f745")]
    vehicle: String,

    /// Start in offline Mock Drone mode even if a USB port is present.
    #[arg(long)]
    mock: bool,
}

fn main() -> eframe::Result<()> {
    let args = Args::parse();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 820.0])
            .with_min_inner_size([960.0, 620.0])
            .with_title("Cybflight Configurator & Ground Station"),
        ..Default::default()
    };

    eframe::run_native(
        "Cybflight Configurator",
        native_options,
        Box::new(move |_cc| Ok(Box::new(CybflightGsApp::new(&args.vehicle, args.mock)))),
    )
}
