from unilabos.devices.bioyond.bioyond import BioyondV1RPC
from unilabos.devices.bioyond.s import res

client = BioyondV1RPC({
        "api_key": "DE9BDDA0",
        "api_host": "http://192.168.1.200:44388",
        })


# print("stock_material result:", client.stock_material({
#                 "typeMode": 1,
#                 "filter":"",
#                 "includeDetail": True,
#             })) # 测试通过
# print("res: ", client.query_workflow(
#     {
#         "type": 2,
#         "includeDetail": True,
#     }
# )) # 测试通过
# print("res: ", client.workflow_step_query(workflow_id="3a16082a-a26a-273c-e375-9bb9d075f546")) # 测试通过
# print("res: ", client.create_order([{
#     "orderCode": "test12345678915",
#     "orderName": "test12345678915",
#     "workFlowId": "3a1a0ea1-a0b3-f5af-190c-198a36f43e7f",
#     "borderNumber": 1,
#     "paramValues": res}]
# ))  # 测试通过
# print("res: ", client.step_finish()) # 调度系统推送到 uni lab 服务端，数据格式就是参数。
# print("res: ", client.preintake_finish()) # 调度系统推送到 uni lab 服务端，数据格式就是参数。
# print("res: ", client.order_finish()) # 调度系统推送到 uni lab 服务端，数据格式就是参数。
# print("res: ", client.order_query({
#     "beginTime":"2025-05-23T08:00:00.931Z",
#     "endTime":"2025-05-23T09:55:54.931Z",
#     "timeType":"CreationTime",
#     "status": "60",
#     "skipCount": 0,
#     "pageCount": 10,
# })) # 测试通过
# print("res: ", client.order_report(order_id="3a19591c-429a-7ce2-50e9-2f063e18bc62")) # 测试通过
# print("res: ", client.order_takeout({
#     "orderId": "3a1a0e57-2101-c6b2-4602-4f33debd24a0"
# }))# 测试通过
# print("res: ", client.device_list()) # 没设备
# print("res: ", client.device_operation()) # 没设备
# print("res: ", client.scheduler_status()) # 测试通过
# print("res: ", client.scheduler_start()) # 测试通过
# print("res: ", client.scheduler_pause())  # 测试通过
# print("res: ", client.scheduler_continue()) # 测试通过
print("res: ", client.scheduler_stop()) # 测试通过
# print("res: ", client.scheduler_reset()) # 测试通过
# print("res: ", client.query_simulation(order_id='944d9907-347d-4d2a-86ba-a4ab732feefc'))
# print("res: ", client.query_simulation_gantts())
# print("res: ", client.query_run_gantts(order_id='3a1a0d85-4213-700e-d972-d5aa553d2421')) # 测试通过
# print("res: ", client.cancel_order(order_id="test12345678903")) # 测试通过
# print("res: ", client.query_material_type()) # 测试通过
# print("res: ", client.create_material(
#     {
#         "typeId": "3a142339-80de-8f25-6093-1b1b1b6c322e",
#         "code": "test__0002",
#         "barCode": "",
#         "name":"test",
#         "unit": "test",
#         "quantity": 1
#     }
# ))  # 测试通过
# print("res: ", client.delete_material(material_id="3a1a0a30-3c39-2881-3be4-80fe25d8b04b")) # 测试通过
# print("res: ", client.query_warehouse({"typeId": "3a142339-80de-8f25-6093-1b1b1b6c322e"})) # 测试通过
# print("res: ", client.material_inbound({
#     "materialId":"3a1a0a33-1948-5f81-2117-f871ff5fc112",
#     "locationId": "3a14aa17-0d49-9c7d-1145-d554a6e482f0"
# })) # 测试通过
# print("res: ", client.material_outbound(
#     {
#     "materialId":"3a1a0a33-1948-5f81-2117-f871ff5fc112",
#     "locationId": "3a14aa17-0d49-9c7d-1145-d554a6e482f0",
#     "quantity":1, 
#     }
# )) # 测试通过
# print("res: ", client.query_split_workflow()) # 测试通过
# print("res: ", client.merge_workflow({
#     "name":"4ABBBCDDDDDDGF-0418_test",
#     "workflowIds":[
# "3a160df6-76b3-0957-9eb0-cb496d5721c6",
# "3a16087e-124f-8ddb-8ec1-c2dff09ca784",
# "3a16087e-124f-8ddb-8ec1-c2dff09ca784",
# "3a16087e-124f-8ddb-8ec1-c2dff09ca784",
# "3a160877-87e7-7699-7bc6-ec72b05eb5e6",
# "3a16082a-96ac-0449-446a-4ed39f3365b6",
# "3a16082a-96ac-0449-446a-4ed39f3365b6",
# "3a16082a-96ac-0449-446a-4ed39f3365b6",
# "3a16082a-96ac-0449-446a-4ed39f3365b6",
# "3a16082a-96ac-0449-446a-4ed39f3365b6",
# "3a162cf9-6aac-565a-ddd7-682ba1796a4a",
# "3a16081e-4788-ca37-eff4-ceed8d7019d1"
#     ]
# })) # 测试通过
# print("res: ", client.start_observe(
#         {
#                 "preIntakeId":"3a1a0ea5-e656-2822-b538-5df99535ac6b",
#                 "stepIndex":"0",
#         }
# ))
# print("res: ", client.finish_observe({
#         "preIntakeId": "3a1a0ea5-e656-2822-b538-5df99535ac6b",
#         "result": "0",
#         "stepIndex": "0"
# }))
