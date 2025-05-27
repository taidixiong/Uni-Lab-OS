from enum import Enum
from datetime import datetime, timezone
from unilabos.device_comms.rpc import BaseRequest
from typing import Optional


class MachineState(Enum):
    INITIAL = 0
    STOPPED = 1
    RUNNING = 2
    PAUSED = 3
    ERROR_PAUSED = 4
    ERROR_STOPPED = 5


class MaterialType(Enum):
    Consumables = 0  # 耗材
    Sample = 1  # 样品
    Reagent = 2  # 试剂
    Product = 3  # 成品


class BioyondV1RPC(BaseRequest):
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.api_key = config["api_key"]
        self.host = config["api_host"]

    def get_current_time_iso8601(self) -> str:
        """
        获取当前时间，并格式化为 ISO 8601 格式（包含毫秒部分）。

        :return: 当前时间的 ISO 8601 格式字符串
        """
        current_time = datetime.now(timezone.utc).isoformat(
            timespec='milliseconds'
        )
        # 替换时区部分为 'Z'
        current_time = current_time.replace("+00:00", "Z")
        return current_time

    # 物料查询接口
    def stock_material(self, params: dict) -> list:
        """
            描述：返回所有当前在库的，已启用的物料
            params 字段介绍
            {
                typeMode: 物料类型, 样品1、试剂2、耗材0
                filter: 过滤字段, 物料名称/物料编码
                includeDetail: 是否包含所在库位。true，false
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/storage/stock-material',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"query_material error: {response.get('message', '')}"
            )
            return []

        self.get_logger().info(f"query_material data: {response['data']}")
        return response.get("data", [])

    # 工作流列表查询
    def query_workflow(self, params: dict) -> dict:
        """
            描述：查询工作流列表，仅显示当前可用于新建实验的工作流
            params 字段介绍
            {
                type: 工作流类型, 0常规工作流/1维护工作流/2模块工作流
                filter: 过滤字段，可过滤工作流名称、代码。字符串
                includeDetail: 是否包含子工作流。Bool值, true/fase
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/workflow/work-flow-list',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return {}

        if response['code'] != 1:
            self.get_logger().error(
                f"query_workflow error: {response.get('message')}"
            )
            return {}

        self.get_logger().info(f"query_workflow data: {response['data']}")
        return response.get("data", {})

    # 工作流步骤查询接口
    def workflow_step_query(self, workflow_id: str) -> dict:
        """
            描述：查询某一个子工作流的详细信息，包含所有步骤、参数信息
            workflow_id: 子工作流 id
        """
        response = self.post(
            url=f'{self.host}/api/lims/workflow/sub-workflow-step-parameters',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": workflow_id,
            })

        if not response:
            return {}

        if response['code'] != 1:
            self.get_logger().error(
                f"query_workflow error: {response.get('message')}"
            )
            return {}

        self.get_logger().info(f"workflow_step_query data: {response['data']}")
        return response.get("data", {})

    # 任务推送接口
    def create_order(self, params: list) -> dict:
        """
            描述：新建并开始任务，返回需要的物料和入库的库位
            params
            {
                code : 任务编码。字符串
                Name: 任务名称。字符串
                workflowName: 工作流名称。字符串
                borderNumber: 通量数。 整数
                paramValues:{//参数
                id1:[{
                m: 步骤标识1(泳道)。整数
                n: 步骤标识2(步骤)。整数
                key: 关键词。字符串
                vaue: 值。
                },{}....],
                id2:[{},{}],
                id3:[{},{}]
                }
            }
        """
        import json
        print('===============', json.dumps(params))
        response = self.post(
            url=f'{self.host}/api/lims/order/order',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return {}

        if response['code'] != 1:
            self.get_logger().error(
                f"create order error: {response.get('message')}"
            )
            return {}

        self.get_logger().info(f"create order data: {response['data']}")
        return response.get("data", {})

    # 步骤完成接口、每一步骤完成后，推送给第三方 FIXME 暂时参数不明
    def step_finish(self, params: dict) -> list:
        """
            描述：每一步骤执行完成后，推送给第三方
            补充说明： uni lab 服务端提供接口，数据推送到服务端，服务端路径是 /Lims/step_finish
            params
            {
                orderCode: 任务号。字符串
                orderName:任务名称。字符串
                stepName:步骤名称。字符串
                stepId:步骤Id。GUID
                sampleId:通量Id。GUID
                startTime:开始时间。时间格式
                endTime:完成时间。时间格式
            }
        """
        response = self.post(
            url=f'{self.host}/LIMS/step_finish',
            params={
                "授权字段 FIXME": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"step_complete error: {response.get('message')}"
            )
            return []

        self.get_logger().info(f"step_complete data: {response['data']}")
        return response.get("data", {})

    # 通量完成接口，每一通量完成后，推送给第三方 FIXME 暂时参数不明
    def preintake_finish(self, params: dict) -> list:
        """
            描述：每一通量完成后，推送给第三方
            params
            {
                "orderCode": "任务号",
                "orderName": "任务名称",
                "sampleId":"通量Id",
                "startTime":"开始时间",
                "endTime":"完成时间",
                "status":"完成状态"
            }
        """
        response = self.post(
            url=f'{self.host}/LIMS/preintake_finish',
            params={
                "授权字段 FIXME": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"flux_complete error: {response.get('message')}"
            )
            return []

        self.get_logger().info(f"flux_complete data: {response['data']}")
        return response.get("data", {})

    # 任务完成接口， 试验的都有通量都完成后，推送给第三方 FIXME 暂时参数不明
    def order_finish(self, params: dict) -> list:
        """
            描述：实验的所有通量都完成后，推送给第三方
            params
            {
                "orderCode": "任务号",
                "orderName": "任务名称",
                "startTime":"开始时间",
                "endTime":"完成时间",
                "status":"完成状态",
                "usedMaterials":[{
                    "materialId":"物料Id",
                    "locationId":"库位Id",
                    "typeMode":"物料类型",
                "usedQuantity":"使用数量"
                },{}]
            }
        """
        response = self.post(
            url=f'{self.host}/LIMS/order_finish',
            params={
                "授权字段 FIXME": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"order finish error: {response.get('message')}"
            )
            return []

        self.get_logger().info(f"order finish data: {response['data']}")
        return response.get("data", {})

    # 任务列表查询接口，查询任务列表
    def order_query(self, params: dict) -> dict:
        """
            描述：查询任务列表
            params
            {
                "timeType"：查询时间类型。创建时间：CreationTime，完成时间：FinishedTime
                "beginTime":开始时间。时间格式
                "endTime":结束时间。时间格式，必须大于beginTime
                "status":状态, 成功80/失败90/执行中60
                "filter":查询字段，查询任务编码和任务名称。字符串
                "skipCount":"起始数",
                "pageCount":"数量",
                "sorting":"排序字段",
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/order-list',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return {}

        if response['code'] != 1:
            self.get_logger().error(
                f"order_query error: {response.get('message')}"
            )
            return {}

        self.get_logger().info(f"order_query data: {response['data']}")
        return response.get("data", {})

    # 任务明细查询，查询某个任务明细
    def order_report(self, order_id: str) -> dict:
        """
            描述：查询某个任务明细
            order_id: 任务 id
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/order-report',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": order_id,
            })

        if not response:
            return {}

        if response['code'] != 1:
            self.get_logger().error(
                f"order report error: {response.get('message')}"
            )
            return {}

        self.get_logger().info(f"order report data: {response['data']}")
        return response.get("data", {})

    # 样品/废料取出接口，在试验完成后，取出样品、废料、废液等
    def order_takeout(self, params: dict) -> int:
        """
            描述：在实验完成后，取出样品、废料、废液等
            params
            {
                "orderId":"实验Id",
                "preintakeIds":"通量Id列表, 可为空",
                "materialIds":"物料Id列表, 可为空"
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/take-out',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"order takeout error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"order takeout data: {response['code']}")
        return response.get("code", 1)

    # 设备列表，获取设备列表，包含设备操作指令详情
    def device_list(self, device_no: Optional[str] = None) -> list:
        """
            描述：获取设备列表，包含设备操作指令详情
        """
        response = self.post(
            url=f'{self.host}/api/lims/device/device-list',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": device_no,
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"device_query error: {response.get('message')}"
            )
            return []

        self.get_logger().info(f"device_query data: {response['data']}")
        return response.get("data", [])

    # 设备操作 exec-operation , 给设备发送操作命令
    def device_operation(self, params: dict) -> int:
        """
            描述：给设备发送操作指令
            params
            {
                cmd: 设备操作指令,
                description: 指令描述,
                index: 指令序号,
                parameters :[{ 参数列表
                Name: 参数名,
                Value: 参数值,
                description: 描述,
                valueType: 参数类型,
                enumKeys : 枚举显示值,
                enumValues: 枚举值,
                isNeedRange : 是否控制范围,
                minValue: 最小值,
                maxValue: 最大值
                },{}]
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/device/execute-operation',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"device_operation error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"device_operation data: {response['code']}")
        return response.get("code", 1)

    # 查看调度状态
    def scheduler_status(self) -> dict:
        """
            描述： 查看调度状态
        """
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/scheduler-status',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response:
            return {}

        if response['code'] != 1:
            self.get_logger().error(
                f"scheduler status error: {response.get('message')}"
            )
            return {}

        self.get_logger().info(f"scheduler status data: {response['code']}")
        return response.get("data", {})

    # 启动调度 scheduler start
    def scheduler_start(self) -> int:
        """
            描述：启动调度
        """
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/start',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"scheduler start error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"scheduler start data: {response['code']}")
        return response.get("code", 0)

    # 暂停调度
    def scheduler_pause(self) -> int:
        """
            描述：暂停调度
        """
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/pause',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"scheduler pause error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"scheduler pause data: {response['code']}")
        return response.get("code", 0)

    # 继续调度
    def scheduler_continue(self) -> int:
        """
            描述：继续调度
        """
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/continue',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"scheduler pause error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"scheduler pause data: {response['code']}")
        return response.get("code", 0)

    # 停止调度
    def scheduler_stop(self) -> int:
        """
            描述：停止调度
        """
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/stop',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"scheduler stop error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"scheduler stop data: {response['code']}")
        return response.get("code", 0)

    # 复位调度
    def scheduler_reset(self) -> int:
        """
            描述：停止调度
        """
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/reset',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"scheduler stop error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"scheduler stop data: {response['code']}")
        return response.get("code", 0)

    # 查询仿真
    def query_simulation(self, order_id: str) -> list:
        """
            描述：查询仿真Gantt信息
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/simulation-gantt-by-order-id',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": order_id,
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"query simulation error: {response.get('message')}"
            )
            return []

        self.get_logger().info(f"query simulation data: {response['code']}")
        return response.get("data", [])

    # 查询仿真运行 gantt
    def query_simulation_gantts(self, order_id: str) -> list:
        """
            描述：查询仿真Gantt信息
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/gantts-by-order-id',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": order_id,
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"query simulation gantts error: {response.get('message')}"
            )
            return []

        self.get_logger().info(
            f"query simulation gantts data: {response['code']}"
            )
        return response.get("data", [])

    # 查询运行 gantt
    def query_run_gantts(self, order_id: str) -> list:
        """
            描述：查询实验Gantt信息
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/gantts-by-order-id',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": order_id,
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"query run gantts error: {response.get('message')}"
            )
            return []

        self.get_logger().info(f"query run gantts data: {response['code']}")
        return response.get("data", [])

    # 取消任务
    def cancel_order(self, order_id: str) -> bool:
        """
            描述：取消任务
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/cancel-experiment',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": order_id,
            })

        if not response:
            return False

        if response['code'] != 1:
            self.get_logger().error(
                f"cancel task error: {response.get('message')}"
            )
            return False

        self.get_logger().info(f"cancel task data: {response['code']}")
        return response.get("data", False)

    # 查询物料类型
    def query_material_type(self, key_words: Optional[str] = "") -> list:
        """
            描述：查询物料类型
            key_words
            物料类型的代码或名称
        """
        response = self.post(
            url=f'{self.host}/api/lims/storage/material-types',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": key_words,
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"query materila type error: {response.get('message')}"
            )
            return []

        self.get_logger().info(f"query material type data: {response['code']}")
        return response.get("data", [])

    def create_material(self, params: dict) -> int:
        """
            描述：添加新的物料
            params
            {
                "typeId": "物料类型Id",
                "code": "物料编码",
                "barCode": "物料条码",
                "name": "物料名称",
                "unit": "单位",
                "quantity": 0, //数量
                "details": [//孔物料信息
                    {
                        "typeId": "物料类型Id",
                        "code": "物料编码",
                        "name": "物料名称",
                        "quantity": 0, //数量
                        "x": 0, //孔坐标X
                        "y": 0, //孔坐标Y
                        "z": 0, //孔坐标Z
                        "unit": "单位"
                    }
                ]
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/storage/material',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"query materila type error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"query material type data: {response['code']}")
        return response.get("data", 0)

    # 删除物料
    def delete_material(self, material_id: str) -> int:
        """
            描述：删除尚未入库的物料
            material_id
            物料 id
        """
        response = self.post(
            url=f'{self.host}/api/lims/storage/delete-material',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": material_id,
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"delete material error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"delete material data: {response['code']}")
        return response.get("data", 0)

    # 查询物料关联库位
    def query_warehouse(self, params: dict) -> list:
        """
            描述：查询物料类型可以入库的库位
            params
            {
                "typeId": "" //物料类型Id
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/storage/warehouse-info-by-mat-type-id',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"query warehouse error: {response.get('message')}"
            )
            return []

        self.get_logger().info(f"query warehouse data: {response['code']}")
        return response.get("data", [])

    # 物料入库
    def material_inbound(self, params: dict) -> int:
        """
            描述：指定库位入库一个物料
            params
            {
                "materialId":"物料Id",
                "locationId":"库位Id"
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/storage/inbound',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"material inbound error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"material inbound data: {response['code']}")
        return response.get("code", 0)

    # 物料出库
    def material_outbound(self, params: dict) -> int:
        """
            描述：指定库位入库一个物料
            params
            {
                "materialId":"物料Id",
                "locationId":"库位Id",
                "quantity":1, //数量
                "detail":{
                        "x":1,//孔坐标X
                        "y":1, //孔坐标Y
                        "z":1, //孔坐标Z
                        "quantity":1 //数量
                },
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/storage/outbound',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"material outbound error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(f"material outbound data: {response['code']}")
        return response.get("code", 0)

    # 获取可拼接工作流
    def query_split_workflow(self) -> list:
        """
            描述：获取可用于拼接的工作流列表
        """
        response = self.post(
            url=f'{self.host}/api/lims/workflow/split-workflow-list',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response:
            return []

        if response['code'] != 1:
            self.get_logger().error(
                f"query split workflow error: {response.get('message')}"
            )
            return []

        self.get_logger().info(
            f"query split workflow data: {response['code']}"
            )
        return response.get("data", [])

    # 合并工作流
    def merge_workflow(self, params: dict) -> dict:
        """
            描述：多个子工作流拼接成一个新工作流
            params
            {
                "name":"新工作流名称",
                "workflowIds":["",""]
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/workflow/merge-workflow',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response:
            return {}

        if response['code'] != 1:
            self.get_logger().error(
                f"merge workflow error: {response.get('message')}"
            )
            return {}

        self.get_logger().info(
            f"merge workflow data: {response['code']}"
            )
        return response.get("data", {})

    # 启动观察
    def start_observe(self, params: dict) -> int:
        """
            描述：启动该通量的观察
            params
            {
                "preIntakeId":"通量Id",
                "stepIndex":"步骤序号",默认0
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/start-observe',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"start observe error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(
            f"start observe data: {response['code']}"
            )
        return response.get("data", 0)

    # 完成观察
    def finish_observe(self, params: dict) -> int:
        """
            描述：完成该通量的观察，并且输入观察结果（继续实验/停止实验）
            params
            {
                "preIntakeId":"通量Id",
                "result":结果 //0停止实验, 1继续实验
                "stepIndex":"开始步骤序号",
            }
        """
        response = self.post(
            url=f'{self.host}/api/lims/order/finish-observe',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response:
            return 0

        if response['code'] != 1:
            self.get_logger().error(
                f"finish observe error: {response.get('message')}"
            )
            return 0

        self.get_logger().info(
            f"finish observe data: {response['code']}"
            )
        return response.get("data", 0)


if __name__ == "__main__":
    bioyong = BioyondV1RPC({
        "api_key": "DE9BDDA0",
        "api_host": "http://192.168.1.200:44388",
        })
    print('device list==================\n', bioyong.device_list())
    print('scheduler status=============\n', bioyong.scheduler_status())
    print('query split workflow=========\n', bioyong.query_split_workflow())
