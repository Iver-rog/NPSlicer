use thiserror::Error;

#[derive(Debug, Error)]
pub enum SkeletonError {
    #[error("Invalid polygon: {0}")]
    InvalidPolygon(String),
    #[error("Computation error: {0}")]
    ComputationError(String),
    #[error("Bisector Calculation Erorr: {0}")]
    BisectorError(String),
    #[error("Initialization Error could not create SkeletonBuilder {0}")]
    InitializationError(String),
    #[error("Split event Error {0}")]
    EdgeEventError(String),
    #[error("Split event Error {0}")]
    SplitEventError(String),
    }
