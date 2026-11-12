"""Local-only main recording pipeline for recovered DAgger integration tests."""
from middleware.core.platform_connection import PlatformConnection
from middleware.core.task_recording import TaskRecorder
from middleware.core.topic_recorder import TopicRecorder


def config(directory):
    return dict(directory=str(directory / 'data'), format='mcap',
                artifact_state_dir=str(directory / 'artifacts'),
                upload_job_state_dir=str(directory / 'uploads'), min_free_bytes=0,
                corrections=dict(prepared_slots=2, late_message_sec=.05))


def service(recording_config, config_dir, ros_config):
    cfg = dict(ros={**ros_config, 'recording': recording_config},
               uploads=dict(queue_directory=str(config_dir / 'queue')))
    recorder = TopicRecorder(recording_config, config_dir)
    platform = PlatformConnection(cfg, config_dir, recorder.directory)
    # No connection or upload worker is started in these tests.
    result = TaskRecorder(recorder, platform)
    result.configure_corrections(cfg['ros'])
    return result


def local_service(directory):
    return service(config(directory), directory, dict(enabled=False, subscriptions=[]))
