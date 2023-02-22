use async_trait::async_trait;
use flume::Receiver;
use std::sync::Arc;
use zenoh::{prelude::r#async::*, subscriber::Subscriber};
use zenoh_flow::prelude::*;

static PORT_ID: &str = "Data";
static KE: &str = "scan";

#[export_source]
pub struct ZenohSource<'a> {
    _session: Arc<Session>,
    data: OutputRaw,
    subscriber: Subscriber<'a, Receiver<Sample>>,
}

#[async_trait]
impl<'a> Source for ZenohSource<'a> {
    async fn new(
        context: Context,
        configuration: Option<Configuration>,
        mut outputs: Outputs,
    ) -> Result<Self> {
        let data = outputs
            .take_raw(PORT_ID)
            .ok_or(zferror!(ErrorKind::NotFound, "Output Data not found!!"))?;

        let ke = match configuration {
            Some(configuration) => match configuration.get("subscriber") {
                Some(ke) => match ke.as_str() {
                    Some(ke) => ke.to_string(),
                    None => KE.to_string(),
                },
                None => KE.to_string(),
            },
            None => KE.to_string(),
        };

        let subscriber = context.zenoh_session().declare_subscriber(&ke).res().await?;
        Ok(ZenohSource {
            _session: context.zenoh_session(),
            data,
            subscriber,
        })
    }
}

#[async_trait]
impl<'a> Node for ZenohSource<'a> {
    async fn iteration(&self) -> Result<()> {
        let sample = self.subscriber.recv_async().await?;
        let data = sample.payload.contiguous().to_vec();
        self.data.send(data, None).await?;

        Ok(())
    }
}
