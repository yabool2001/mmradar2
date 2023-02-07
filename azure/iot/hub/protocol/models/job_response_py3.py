# coding=utf-8
# --------------------------------------------------------------------------
# Code generated by Microsoft (R) AutoRest Code Generator.
# Changes may cause incorrect behavior and will be lost if the code is
# regenerated.
# --------------------------------------------------------------------------

from msrest.serialization import Model


class JobResponse(Model):
    """JobResponse.

    :param job_id: System generated.  Ignored at creation. The unique
     identifier of the job.
    :type job_id: str
    :param query_condition: The device query condition.
    :type query_condition: str
    :param created_time: System generated.  Ignored at creation. The creation
     date and time of the job.
    :type created_time: datetime
    :param start_time: The start date and time of the scheduled job in UTC.
    :type start_time: datetime
    :param end_time: System generated.  Ignored at creation. The end date and
     time of the job in UTC.
    :type end_time: datetime
    :param max_execution_time_in_seconds: The maximum execution time in
     secounds.
    :type max_execution_time_in_seconds: long
    :param type: The job type. Possible values include: 'unknown', 'export',
     'import', 'backup', 'readDeviceProperties', 'writeDeviceProperties',
     'updateDeviceConfiguration', 'rebootDevice', 'factoryResetDevice',
     'firmwareUpdate', 'scheduleDeviceMethod', 'scheduleUpdateTwin',
     'restoreFromBackup', 'failoverDataCopy'
    :type type: str or ~protocol.models.enum
    :param cloud_to_device_method: The method type and parameters. This is
     required if job type is cloudToDeviceMethod.
    :type cloud_to_device_method: ~protocol.models.CloudToDeviceMethod
    :param update_twin:
    :type update_twin: ~protocol.models.Twin
    :param status: System generated.  Ignored at creation. The status of the
     job. Possible values include: 'unknown', 'enqueued', 'running',
     'completed', 'failed', 'cancelled', 'scheduled', 'queued'
    :type status: str or ~protocol.models.enum
    :param failure_reason: The reason for the failure, if a failure occurred.
    :type failure_reason: str
    :param status_message: The status message of the job.
    :type status_message: str
    :param device_job_statistics: The details regarding job execution status.
    :type device_job_statistics: ~protocol.models.DeviceJobStatistics
    """

    _attribute_map = {
        "job_id": {"key": "jobId", "type": "str"},
        "query_condition": {"key": "queryCondition", "type": "str"},
        "created_time": {"key": "createdTime", "type": "iso-8601"},
        "start_time": {"key": "startTime", "type": "iso-8601"},
        "end_time": {"key": "endTime", "type": "iso-8601"},
        "max_execution_time_in_seconds": {"key": "maxExecutionTimeInSeconds", "type": "long"},
        "type": {"key": "type", "type": "str"},
        "cloud_to_device_method": {"key": "cloudToDeviceMethod", "type": "CloudToDeviceMethod"},
        "update_twin": {"key": "updateTwin", "type": "Twin"},
        "status": {"key": "status", "type": "str"},
        "failure_reason": {"key": "failureReason", "type": "str"},
        "status_message": {"key": "statusMessage", "type": "str"},
        "device_job_statistics": {"key": "deviceJobStatistics", "type": "DeviceJobStatistics"},
    }

    def __init__(
        self,
        *,
        job_id: str = None,
        query_condition: str = None,
        created_time=None,
        start_time=None,
        end_time=None,
        max_execution_time_in_seconds: int = None,
        type=None,
        cloud_to_device_method=None,
        update_twin=None,
        status=None,
        failure_reason: str = None,
        status_message: str = None,
        device_job_statistics=None,
        **kwargs
    ) -> None:
        super(JobResponse, self).__init__(**kwargs)
        self.job_id = job_id
        self.query_condition = query_condition
        self.created_time = created_time
        self.start_time = start_time
        self.end_time = end_time
        self.max_execution_time_in_seconds = max_execution_time_in_seconds
        self.type = type
        self.cloud_to_device_method = cloud_to_device_method
        self.update_twin = update_twin
        self.status = status
        self.failure_reason = failure_reason
        self.status_message = status_message
        self.device_job_statistics = device_job_statistics
