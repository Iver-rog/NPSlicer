use iced::widget::text::Highlighter;
use npslicer_core::{self,slice};

mod scene;
use scene::{Object, Scene};

mod io;
use io::{pick_input_file,pick_output_file};

mod widgets;
use widgets::{pill_button, text_button, card, number_input, NumberInput, gcode_view, GcodeView};

use std::path::PathBuf;
use std::fmt;

mod test_gcode;
use test_gcode::TEST_GCODE;

use iced::wgpu;
use iced::widget::{checkbox, column, row, shader, text, button, container, pick_list, space, stack, slider};
use iced::widget::text::Span;
use iced::{Bottom, Color, Element, Fill, Length, Right, Center, Top, Shrink};
use iced::task::Task;

use env_logger;

fn slice_w_progress(
        path:PathBuf,
        settings:npslicer_core::Settings,
    ) -> impl iced::task::Sipper<Result<String,Error>, f32 >{
    iced::task::sipper(async move |mut sender|{

        let (tx, mut rx) = tokio::sync::mpsc::channel::<f32>(1);
        let handle = tokio::task::spawn_blocking(move ||{ 
            let _ = tx.try_send(0.0);
            npslicer_core::slice(path,settings,tx)
        });
        while let Some(pct) = rx.recv().await {
            sender.send(pct).await;
        }
        handle.await.map_err(|tokio_error|{
            eprintln!("{tokio_error}");
            Error::SlicerCrashed
        })

    })
}

fn main() -> iced::Result {
    unsafe { std::env::set_var("WGPU_POWER_PREF", "low") };
    env_logger::init();
    iced::application(Controls::default, Controls::update, Controls::view)
        .title("layer-gen-rs")
        // .executor::<iced::executor::Default>()
        .theme(iced::Theme::Nord)
        .run()
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Printer{
    name: &'static str,
    print_bed_path: &'static str,
}
impl Printer {
    fn get_printers() -> [Printer;3] {
        [
            Printer{
                name: "PrusaMK3S+",
                print_bed_path: "/home/iver/Documents/NTNU/Master/layer-gen-rs/printbeds/mk3_bed.stl",
            },
            Printer{
                name: "Prusa MINI",
                print_bed_path: "/home/iver/Documents/NTNU/Master/layer-gen-rs/printbeds/mini_bed.stl",
            },
            Printer{
                name: "Prusa XL",
                print_bed_path: "/home/iver/Documents/NTNU/Master/layer-gen-rs/printbeds/Prusa XL_bed.stl",
            }
        ]
    }
}
impl fmt::Display for Printer{
    fn fmt(&self, f: &mut fmt::Formatter)-> fmt::Result {
        write!(f,"{}",self.name)
    }
}
impl Default for Printer {
    fn default() -> Self {
        Self{
            name: "PrusaMK3S+",
            print_bed_path: "/home/iver/Documents/NTNU/Master/layer-gen-rs/printbeds/mk3_bed.stl",
        }
    }
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Filament{
    PLA,
    Other(String),
}
impl fmt::Display for Filament{
    fn fmt(&self, f: &mut fmt::Formatter<'_>)-> fmt::Result{
        match self{
            Self::PLA => write!(f,"PLA"),
            Self::Other(filament_name) => write!(f,"{filament_name}"),
        }
    }
}
impl Default for Filament {
    fn default() -> Self {
        Self::PLA
    }
}
pub struct Parameters {
    overhang_angle: NumberInput<f32>,
    brim: usize,
    nr_of_perimeters: usize,
    layer_height: f32,
    infill_percentage: usize,
}
impl Default for Parameters{
    fn default() -> Self {
        Self{
            overhang_angle: NumberInput::new(20.0),
            brim: 0,
            nr_of_perimeters: 2,
            layer_height: 0.4,
            infill_percentage: 20,
        }
    }
}

struct Controls {
    scene: Scene,
    inputstl: Option<PathBuf>,
    printers: Option<Printer>,
    filament: Option<Filament>,
    parameters: Parameters,
    slicing_progress: Option<f32>,
    notifications: Vec<Notification>,
    gcode: Option<GcodeView>,
    gcode_layer_nr: u32,
    gcode_line_nr: u32,
}
#[derive(Debug,Clone)]
struct Notification {
    kind: NotificationType,
    message: String,
}
#[derive(Debug,Clone)]
enum NotificationType{
    Error,
    Warning,
    Info
}
impl From<Error> for Notification{
    fn from(error:Error) -> Self {
        Self{
            kind: NotificationType::Error,
            message: format!("{error}")
        }
    }
}
impl fmt::Display for Notification{
    fn fmt(&self,b: &mut fmt::Formatter)-> fmt::Result{
        write!(b,"{}",self.message)
    }
}

#[derive(Debug, Clone)]
enum Message {
    Err(Error),
    Nofify(Notification),
    DismissNotification(usize),
    Camera(scene::camera::CameraEvent),
    ShowDepthBuffer(bool),
    ModelColorChanged(Color),
    PrinterChanged(Printer),
    FilamentChanged(Filament),
    NewModel((Object,PathBuf)),
    SliceModel,
    SlicerProgress(f32),
    SlicingComplete(String),
    PickInputFile,
    PickOutputFile,
    OverhangAngleChanged(String),
    OverhangAngleUpdated,
    GcodeLayerNrChanged(u32),
    GcodeLineNrChanged(u32),
}

impl Controls {
    fn new() -> Self {
        Self {
            scene: Scene::new(),
            printers: Some(Printer::default()),
            filament: Some(Filament::default()),
            parameters: Parameters::default(),
            inputstl: None,
            slicing_progress: None,
            notifications: Vec::new(),
            gcode: None,
            // gcode: Some(GcodeView::new(TEST_GCODE.into())),
            gcode_layer_nr:0,
            gcode_line_nr:0,
        }
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::SlicerProgress(progress) => { 
                self.slicing_progress = Some(progress); 
                Task::none() 
            },
            Message::Err(error) => { self.notifications.push(error.into()); Task::none() },
            Message::Nofify(notification) => { self.notifications.push(notification); Task::none() },
            Message::DismissNotification(index) => { self.notifications.remove(index); Task::none() }
            Message::ShowDepthBuffer(show) => {
                self.scene.show_depth_buffer = show;
                Task::none()
            }
            Message::ModelColorChanged(color) => {
                self.scene.model_color = color;
                Task::none()
            }
            Message::PrinterChanged(printer)     => {
                self.printers = Some(printer.clone());
                let file = std::fs::File::open(&printer.print_bed_path).unwrap();
                let mut reader = std::io::BufReader::new(file);
                self.scene.objects[0].printbed = io::IntoVertexBuffer::into_vertex_buffer(&stl_io::read_stl(&mut reader).unwrap());
                Task::none() 
            }
            Message::OverhangAngleChanged(value) => {self.parameters.overhang_angle.text = value; Task::none()},
            Message::OverhangAngleUpdated => {self.parameters.overhang_angle.commit(); Task::none()},
            Message::FilamentChanged(filament)   => { self.filament = Some(filament); Task::none()}
            Message::SlicingComplete(result)     => { 
                self.gcode = Some(GcodeView::new(result));
                self.slicing_progress = None;
                Task::none() 
            },
            Message::GcodeLineNrChanged(line_nr) => { self.gcode_line_nr = line_nr; Task::none() },
            Message::GcodeLayerNrChanged(layer_nr) => { 
                self.gcode_layer_nr = layer_nr;
                self.gcode_line_nr = self.gcode.as_ref().unwrap().nr_of_lines_in_layer(layer_nr as usize) as u32; 
                Task::none() 
            },
            Message::NewModel((object,path)) => {
                self.scene.new_object(object);
                if let Some(file_name) = path.file_stem(){
                    let str:&str = file_name.to_str().unwrap_or("");
                    self.notifications.push(Notification{
                        kind: NotificationType::Info,
                        message:format!("{}",str)
                    })
                };
                self.inputstl = Some(path);
                Task::none()
            },
            Message::PickInputFile => { 
                Task::future(async{ 
                    match pick_input_file().await {
                        Ok(path) => {
                            match path.extension()
                                .and_then(|ext| ext.to_str())
                                .map(|ext| ext.to_lowercase())
                                .as_deref()
                            {
                                Some("stl") => {
                                    let (object, path) = tokio::task::spawn_blocking(move || {
                                        let file = std::fs::File::open(&path).unwrap();
                                        let mut reader = std::io::BufReader::new(file);
                                        let object = Object::from_mesh(io::IntoVertexBuffer::into_vertex_buffer(&stl_io::read_stl(&mut reader).unwrap()));
                                        (object, path)
                                    }).await.unwrap();

                                    Message::NewModel((object,path))
                                },
                                Some(ext) => panic!("Unsupported file extension: {ext:?}"),
                                None => panic!("No file extension found: {path:?}"),
                            }
                        }
                        Err(error) => Message::Err(error),
                    }
                })
            },
            Message::PickOutputFile => {
                // let gcode = self.gcode.clone().unwrap();
                let gcode = String::from(self.gcode.as_ref().unwrap().as_str());
                Task::future(async move{ 
                    match pick_output_file().await {
                        Err(error) => Message::Err(error),
                        Ok(mut path) => {
                            path.set_extension("gcode");
                            match tokio::fs::write(&path,gcode.as_bytes()).await{
                                Ok(_) => {
                                    Message::Nofify(Notification{
                                        kind: NotificationType::Info,
                                        message: format!("Saved file to {}", path.display())
                                    })
                                },
                                Err(error) => Message::Err(Error::IO(error.kind()))
                            }
                        }
                    }
                })
            },
            Message::SliceModel => { 
                match &self.inputstl {
                    None => Task::none(),
                    Some(path) => {
                        let settings = npslicer_core::Settings::default();
                        let path = path.to_path_buf();
                        Task::sip(
                            slice_w_progress(path,settings),
                            Message::SlicerProgress,
                            |result| match result{
                                Ok(gcode)=>Message::SlicingComplete(gcode),
                                Err(error)=>Message::Err(error)
                            })
                    }
                }
            }
            Message::Camera(camera_event) => {
                self.scene.camera.handle_event(camera_event);
                Task::none()
            }
        }
    }

    fn view(&self) -> Element<'_, Message> {

        let printers = Printer::get_printers();

        let filaments = [
            Filament::PLA,
            Filament::Other("halla mr kis".into()),
            Filament::Other("jalal ".into()),
        ];

        let task_bar = container(row![
            text_button("file").on_press(Message::PickInputFile),
            text_button("settings"),
            space().width(Length::Fill),
            if self.inputstl.is_some(){
                pill_button("Slice").on_press(Message::SliceModel)
            } else { pill_button("Slice") },
            if self.gcode.is_some(){
                pill_button("Export G-code file").on_press(Message::PickOutputFile)
            } else {
                pill_button("Export G-code file")
            },
        ]
        .padding(2)
        .spacing(10)
        )
        .style(container::dark);

        let printer = card(
            row![
                text("Printer"),
                space().width(Length::Fill),
                button("edit").padding(0)
            ],
            column![
                pick_list(printers,self.printers.as_ref(),Message::PrinterChanged).width(Fill),
            ].spacing(4)
        );

        let filament = card(
            row![
                text("Filament"),
                space().width(Length::Fill),
                button("edit").padding(0),
            ],
            column![
                pick_list(filaments,self.filament.as_ref(),Message::FilamentChanged).width(Fill),
            ].spacing(4)
        );

        let process = card(
            row![
                text("Process"),
                space().width(Length::Fill),
                button("edit").padding(0),
            ],
            column![
                control("show depth buffer",
                    checkbox(self.scene.show_depth_buffer)
                        .on_toggle(Message::ShowDepthBuffer)
                ),
                control("overhang angle",
                    number_input(&self.parameters.overhang_angle,Message::OverhangAngleChanged,Message::OverhangAngleUpdated)
                    // .on_input(Message::SlicingComplete)
                ),
                slider(
                    0.0..=10.0,
                    self.scene.model_color.r,
                    |c| Message::ModelColorChanged(Color{r:c/10.0,g:1.0,b:1.0,a:1.0})
                )
            ].spacing(5).height(Fill)
        );

        let side_menu = column![
            printer,
            filament,
            process.height(Fill),
        ].spacing(4)
        .align_x(Right)
        .width(280);


        let notifications = iced::widget::Column::with_children(
                self.notifications.iter()
                    .enumerate()
                    .map(|(id,notification)|
                        button(container( row![text(&notification.message), button("X").on_press(Message::Err(Error::DialogClosed))] )
                            .style(match notification.kind {
                                NotificationType::Error => container::danger,
                                NotificationType::Warning => container::warning,
                                NotificationType::Info => container::primary,
                                }
                            )
                            .padding(0)
                            .width(Fill)
                        ).on_press(Message::DismissNotification(id)).into()
                     )
                ).spacing(5);

        let notifications = if let Some(progress) = self.slicing_progress{
            notifications.push(
                container( column![
                    text(format!("Slicing {}% complete",progress*100.0)),
                    iced::widget::progress_bar(0.0..=1.0,progress)
                ]).style(container::primary).padding(4)
                )
            } else {notifications};


        let gcode_view = if let Some(gcode) = &self.gcode{
            gcode.view(self.gcode_layer_nr,self.gcode_line_nr)
        } else { space().into() };


        let overlay = column![
            gcode_view,
            container(notifications)
                .align_y(Bottom)
                .height(Fill)
                .max_width(250)
            ]
            .align_x(Right)
            .width(Fill)
            .height(Fill)
            .padding([20,0])
            .spacing(20);

        container(
            column![
                task_bar.width(Fill),
                row![
                    side_menu.height(Fill),
                    stack![
                        container(shader(&self.scene).width(Fill).height(Fill)).style(container::bordered_box),
                        column![
                            row![
                                overlay,
                                if let Some(gcode) = &self.gcode{
                                    Element::from(
                                        widgets::vertical_slider(
                                            0..=u32::try_from(gcode.nr_of_layers()-1).unwrap(),
                                            self.gcode_layer_nr,
                                            |n| Message::GcodeLayerNrChanged(n.into())
                                        ).padding([50,5])
                                        .width(50)
                                    )
                                } else { Element::from(space().width(20)) }
                            ],
                            if let Some(gcode) = &self.gcode{
                                Element::from(
                                    widgets::slider(
                                        0..=u32::try_from(gcode.nr_of_lines_in_layer(self.gcode_layer_nr as usize)).unwrap(),
                                        self.gcode_line_nr,
                                        |n| Message::GcodeLineNrChanged(n.into())
                                    ).padding([5,100])
                                )
                            } else { Element::from(space()) }
                        ]
                    ]
                ].spacing(4),
            ],
        ).padding(4)
        .style(container::dark)
        .align_top(Fill)
        .into()
    }
}

impl Default for Controls {
    fn default() -> Self {
        Self::new()
    }
}

fn control<'a>(
    label: &'static str,
    control: impl Into<Element<'a, Message>>,
) -> Element<'a, Message> {
    row![text(label), control.into()].align_y(Center).spacing(10).into()
}


#[derive(Debug, Clone)]
enum Error {
    DialogClosed,
    // SlicerCrashed(tokio::task::JoinError),
    SlicerCrashed,
    IO(std::io::ErrorKind),
}
impl fmt::Display for Error{
    fn fmt(&self, f:&mut fmt::Formatter) -> fmt::Result {
        match &self{
            Error::DialogClosed => write!(f,"File Dialog Closed"),
            Error::SlicerCrashed => write!(f,"Slicer crashed"),
            Error::IO(error) => write!(f,"{error}"),
        }
    }
}

