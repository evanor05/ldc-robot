/* Host-only exercise of the repository's XRCE 2.4.2 code against Lyrical.
 * Uses loopback and migration_check topics; never opens a vehicle transport. */
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define HISTORY 8
#define SIZE (UXR_CONFIG_UDP_TRANSPORT_MTU * HISTORY)
static bool got_twist = false;
static void on_topic(uxrSession *s, uxrObjectId id, uint16_t req,
    uxrStreamId stream, ucdrBuffer *ub, uint16_t len, void *args)
{
    (void)s; (void)id; (void)req; (void)stream; (void)len; (void)args;
    double v[6] = {0};
    bool ok = true;
    for (int i=0; i<6; ++i) ok = ucdr_deserialize_double(ub, &v[i]) && ok;
    if (ok && fabs(v[0]-0.123)<1e-9 && fabs(v[5]-0.321)<1e-9) {
        got_twist = true;
        puts("Received ROS Twist via XRCE 2.4.2"); fflush(stdout);
    }
}
int main(void)
{
    uxrUDPTransport transport;
    if (!uxr_init_udp_transport(&transport, UXR_IPv4, "127.0.0.1", "18888")) return 1;
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0x4D494752);
    uxr_set_topic_callback(&session, on_topic, NULL);
    if (!uxr_create_session(&session)) return 2;
    uint8_t out_buf[SIZE], in_buf[SIZE];
    uxrStreamId out=uxr_create_output_reliable_stream(&session,out_buf,SIZE,HISTORY);
    uxrStreamId in=uxr_create_input_reliable_stream(&session,in_buf,SIZE,HISTORY);
    uxrObjectId participant=uxr_object_id(1,UXR_PARTICIPANT_ID);
    uxrObjectId topic_tx=uxr_object_id(1,UXR_TOPIC_ID), topic_rx=uxr_object_id(2,UXR_TOPIC_ID);
    uxrObjectId pub=uxr_object_id(1,UXR_PUBLISHER_ID), sub=uxr_object_id(1,UXR_SUBSCRIBER_ID);
    uxrObjectId writer=uxr_object_id(1,UXR_DATAWRITER_ID), reader=uxr_object_id(1,UXR_DATAREADER_ID);
    uint16_t requests[7]; uint8_t status[7]={0};
    requests[0]=uxr_buffer_create_participant_xml(&session,out,participant,197,
        "<dds><participant><rtps><name>migration_legacy_client</name></rtps></participant></dds>",UXR_REPLACE);
    const char *string_topic="<dds><topic><name>rt/migration_check/xrce_rx</name><dataType>std_msgs::msg::dds_::String_</dataType></topic></dds>";
    const char *twist_topic="<dds><topic><name>rt/migration_check/xrce_tx</name><dataType>geometry_msgs::msg::dds_::Twist_</dataType></topic></dds>";
    requests[1]=uxr_buffer_create_topic_xml(&session,out,topic_tx,participant,string_topic,UXR_REPLACE);
    requests[2]=uxr_buffer_create_topic_xml(&session,out,topic_rx,participant,twist_topic,UXR_REPLACE);
    requests[3]=uxr_buffer_create_publisher_xml(&session,out,pub,participant,"",UXR_REPLACE);
    requests[4]=uxr_buffer_create_subscriber_xml(&session,out,sub,participant,"",UXR_REPLACE);
    requests[5]=uxr_buffer_create_datawriter_xml(&session,out,writer,pub,
        "<dds><data_writer><topic><kind>NO_KEY</kind><name>rt/migration_check/xrce_rx</name><dataType>std_msgs::msg::dds_::String_</dataType></topic><qos><reliability><kind>RELIABLE</kind></reliability></qos></data_writer></dds>",UXR_REPLACE);
    requests[6]=uxr_buffer_create_datareader_xml(&session,out,reader,sub,
        "<dds><data_reader><topic><kind>NO_KEY</kind><name>rt/migration_check/xrce_tx</name><dataType>geometry_msgs::msg::dds_::Twist_</dataType></topic><qos><reliability><kind>RELIABLE</kind></reliability></qos></data_reader></dds>",UXR_REPLACE);
    if (!uxr_run_session_until_all_status(&session,3000,requests,status,7)) {
        for(int i=0;i<7;++i) fprintf(stderr,"entity %d status %u\n",i,status[i]);
        return 3;
    }
    uxrDeliveryControl control={0}; control.max_samples=UXR_MAX_SAMPLES_UNLIMITED;
    uxr_buffer_request_data(&session,out,reader,in,&control);
    const char *text="legacy-client-to-lyrical-ok";
    for (int i=0; i<50; ++i) {
        ucdrBuffer ub;
        uxr_prepare_output_stream(&session,out,writer,&ub,4+strlen(text)+1);
        ucdr_serialize_string(&ub,text);
        uxr_run_session_timeout(&session,200);
    }
    bool deleted=uxr_delete_session(&session);
    uxr_close_udp_transport(&transport);
    return got_twist && deleted ? 0 : 4;
}
