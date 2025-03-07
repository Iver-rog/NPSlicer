use nalgebra::{Point2,Matrix2};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SkeletonError {
    #[error("Invalid input: Polygon must have at least 3 vetices")]
    InvalidInput,
    #[error("Computation error: {0}")]
    ComputationError(String),
    //#[error("Bisector Calculation Erorr: {0}")]
    #[error("Initialization Error could not initialize SkeletonBuilder: failed to create bisector for node {node}\n{error}")]
    InitializationError{node:usize,error:BisectorError},
    #[error("Split event Error could not create bisector for newly created vertex {0}")]
    EdgeEventError(BisectorError),
    #[error("Split event Error {0}")]
    SplitEventError(String),
    }
#[derive(Debug, Error)]
#[error("Bisector Calculation Error: invertable matrix {matrix} current_point: {current_p} next_point: {next_p} prev_point: {prev_p}")]
pub struct BisectorError {
        pub matrix:Matrix2<f32>,
        pub current_p:Point2<f32>,
        pub next_p:Point2<f32>,
        pub prev_p:Point2<f32>,
    }
