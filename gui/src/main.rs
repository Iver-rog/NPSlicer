use npslicer_core::{self,slice};

mod scene;
use scene::{Object, Scene};

mod io;
use io::pick_file;

mod widgets;
use widgets::{pill_button, text_button, card, number_input, NumberInput};

use std::path::PathBuf;
use std::fmt;

use iced::wgpu;
use iced::widget::{checkbox, column, row, shader, text, button, container, pick_list, space, slider, stack};
use iced::{Color, Element, Length, Bottom, Right, Fill };
use iced::task::Task;

fn main() -> iced::Result {
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
    notifications: Vec<Notification>,
}
struct Notification {
    kind: NotificationType,
    message: String,
}
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
    DismissErr,
    Camera(scene::camera::CameraEvent),
    ShowDepthBuffer(bool),
    ModelColorChanged(Color),
    PrinterChanged(Printer),
    FilamentChanged(Filament),
    NewModel((Object,PathBuf)),
    SliceModel,
    SlicingComplete(()),
    PickFile,
    OverhangAngleChanged(String),
    OverhangAngleUpdated,
}

impl Controls {
    fn new() -> Self {
        Self {
            // ice cubes
            scene: Scene::new(),
            // my controls
            printers: Some(Printer::default()),
            filament: Some(Filament::default()),
            parameters: Parameters::default(),
            inputstl: None,
            notifications: Vec::new(),
        }
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::Err(error) => { self.notifications.push(error.into()); Task::none() },
            Message::DismissErr => { self.notifications = Vec::new(); Task::none() }
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
            Message::SlicingComplete(result)     => { println!("{result:?}"); Task::none() }
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
            // Message::PickFile => { 
            //     Task::perform(pick_file(),
            //         |result| match result {
            //             Ok(path) => {
            //                 match path.extension()
            //                     .and_then(|ext| ext.to_str())
            //                     .map(|ext| ext.to_lowercase())
            //                     .as_deref()
            //                 {
            //                     Some("stl") => {
            //                         let file = std::fs::File::open(&path).unwrap();
            //                         let mut reader = std::io::BufReader::new(file);
            //                         let o = Object::from_mesh(io::IntoVertexBuffer::into_vertex_buffer(&stl_io::read_stl(&mut reader).unwrap()));
            //                         Message::NewModel(o)
            //                     },
            //                     Some(ext) => panic!("Unsupported file extension: {ext:?}"),
            //                     None => panic!("No file extension found: {path:?}"),
            //                 }
            //             }
            //             Err(error) => Message::Err(error),
            //         }
            //     )
            // },
            Message::PickFile => { 
                Task::future(async{ 
                    match pick_file().await {
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
            Message::SliceModel => { 
                match &self.inputstl {
                    Some(path) => {
                        let settings = npslicer_core::Settings::default();
                        let path = path.to_path_buf();
                        Task::future(async{
                            let _ = tokio::task::spawn_blocking(move || {
                                slice(path,settings);
                            }).await.unwrap();
                            Message::SlicingComplete(())
                        })
                    },
                    None => Task::none()
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
            text_button("file").on_press(Message::PickFile),
            text_button("settings"),
            space().width(Length::Fill),
            pill_button("Slice").on_press(Message::SliceModel),
            pill_button("Export G-code file"),
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
                pick_list(printers,self.printers.clone(),Message::PrinterChanged).width(Fill),
            ].spacing(4)
        );

        let filament = card(
            row![
                text("Filament"),
                space().width(Length::Fill),
                button("edit").padding(0),
            ],
            column![
                pick_list(filaments,self.filament.clone(),Message::FilamentChanged).width(Fill),
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

        let shader = shader(&self.scene).width(Fill).height(Fill);
        let overlay = container(
                iced::widget::Column::with_children(
                    self.notifications.iter()
                        .map(|notification|
                            button(container( row![text(&notification.message), button("X").on_press(Message::Err(Error::DialogClosed))] )
                                .style(match notification.kind {
                                    NotificationType::Error => container::danger,
                                    NotificationType::Warning => container::warning,
                                    NotificationType::Info => container::primary,
                                    }
                                )
                                .padding(10)
                                .width(200)
                                // .into()
                            ).on_press(Message::DismissErr).into()
                            // .style(iced::widget::Button{})
                        )
                ).spacing(3)
            )
            .align_x(Right)
            .align_y(Bottom)
            .width(Fill)
            .height(Fill)
            .padding(20);

        container(
            column![
                task_bar.width(Fill),
                row![
                    side_menu.height(Fill),
                    stack![
                        container(shader).style(container::bordered_box),
                        overlay,
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
    row![text(label), control.into()].spacing(10).into()
}


#[derive(Debug, Clone)]
enum Error {
    DialogClosed,
    // IO(ErrorKind),
}
impl fmt::Display for Error{
    fn fmt(&self, f:&mut fmt::Formatter) -> fmt::Result {
        match &self{
            Error::DialogClosed => write!(f,"File Dialog Closed")
        }
    }
}

