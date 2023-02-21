use async_trait::async_trait;
use flume::Receiver;
use std::sync::Arc;
use zenoh::{
    prelude::{r#async::AsyncResolve, SplitBuffer},
    sample::Sample,
    Session, SessionDeclarations,
};
use zenoh_flow::prelude::*;

static PORT_ID: &str = "Data";
static KE: &str = "scan";

#[export_source]
pub struct ZenohSource {
    _session: Arc<Session>,
    data: OutputRaw,
    subscriber: Arc<Receiver<Sample>>,
}

#[async_trait]
impl Source for ZenohSource {
    async fn new(
        context: Context,
        configuration: Option<Configuration>,
        mut outputs: Outputs,
    ) -> Result<Self> {
        let data = outputs
            .take_raw(PORT_ID)
            .ok_or(zferror!(ErrorKind::NotFound, "Output Data not found!!"))?;

        let subscriber = match configuration {
            Some(configuration) => match configuration.get("subscriber") {
                Some(ke) => match ke.as_str() {
                    Some(ke) => context.zenoh_session().declare_subscriber(ke).res().await,
                    None => context.zenoh_session().declare_subscriber(KE).res().await,
                },
                None => context.zenoh_session().declare_subscriber(KE).res().await,
            },
            None => context.zenoh_session().declare_subscriber(KE).res().await,
        }?
        .to_owned();

        Ok(ZenohSource {
            _session: context.zenoh_session(),
            data,
            subscriber: Arc::new(subscriber),
        })
    }
}

#[async_trait]
impl Node for ZenohSource {
    async fn iteration(&self) -> Result<()> {
        let sample = self.subscriber.recv_async().await?;

        self.data
            .send(sample.payload.contiguous().to_vec(), None)
            .await?;

        Ok(())
    }
}
