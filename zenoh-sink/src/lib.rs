use async_trait::async_trait;
use std::sync::Arc;
use zenoh::Session;
use zenoh_flow::prelude::*;
use zenoh_flow::types::LinkMessage;
use zenoh::prelude::r#async::AsyncResolve;

static PORT_ID: &str = "Data";
static KE: &str = "bulb/set";

#[export_sink]
pub struct ZenohSink {
    session: Arc<Session>,
    ke: String,
    data: InputRaw,
}

#[async_trait]
impl Sink for ZenohSink {
    async fn new(
        context: Context,
        configuration: Option<Configuration>,
        mut inputs: Inputs,
    ) -> Result<Self> {
        let data = inputs
            .take_raw(PORT_ID)
            .ok_or(zferror!(ErrorKind::NotFound, "Input Data not found!!"))?;

        let ke = match configuration {
            Some(configuration) => match configuration.get("publisher") {
                Some(ke) => match ke.as_str() {
                    Some(ke) => ke.to_string(),
                    None => KE.to_string(),
                },
                None => KE.to_string(),
            },
            None => KE.to_string(),
        };

        Ok(ZenohSink {
            session: context.zenoh_session(),
            ke,
            data,
        })
    }
}
#[async_trait]
impl Node for ZenohSink {
    async fn iteration(&self) -> Result<()> {
        match self.data.recv().await? {
            LinkMessage::Data(mut dm) => {
                let raw_data = dm.get_inner_data().try_as_bytes()?;
                self.session.put(&self.ke, &**raw_data).res().await?;
            }
            _ => (),
        }

        Ok(())
    }
}
