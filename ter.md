(base) tanbowen@tiantianxiangshangde OpenManus % conda activate open_manus
(open_manus) tanbowen@tiantianxiangshangde OpenManus % python run_flow.py
INFO     [browser_use] BrowserUse logging setup complete with level info
INFO     [root] Anonymized telemetry enabled. See https://docs.browser-use.com/development/telemetry for more information.
Traceback (most recent call last):
  File "/Users/tanbowen/Desktop/OpenManus/run_flow.py", line 4, in <module>
    from app.agent.manus import Manus
  File "/Users/tanbowen/Desktop/OpenManus/app/agent/__init__.py", line 2, in <module>
    from app.agent.browser import BrowserAgent
  File "/Users/tanbowen/Desktop/OpenManus/app/agent/browser.py", line 6, in <module>
    from app.agent.toolcall import ToolCallAgent
  File "/Users/tanbowen/Desktop/OpenManus/app/agent/toolcall.py", line 11, in <module>
    from app.tool import CreateChatCompletion, Terminate, ToolCollection
  File "/Users/tanbowen/Desktop/OpenManus/app/tool/__init__.py", line 8, in <module>
    from app.tool.tool_collection import ToolCollection
  File "/Users/tanbowen/Desktop/OpenManus/app/tool/tool_collection.py", line 7, in <module>
    from logger import logger
ModuleNotFoundError: No module named 'logger'
(open_manus) tanbowen@tiantianxiangshangde OpenManus % python run_flow.py
INFO     [browser_use] BrowserUse logging setup complete with level info
INFO     [root] Anonymized telemetry enabled. See https://docs.browser-use.com/development/telemetry for more information.
Enter your prompt: 制定一个日本七天旅游计划，从中国深圳出发，包含来回返程步骤。衣食住行，旅游景区，风景点，美食，还有其他注意事项
2025-03-27 22:13:18.001 | WARNING  | __main__:run_flow:25 - Processing your request...
2025-03-27 22:13:18.001 | INFO     | app.flow.planning:_create_initial_plan:140 - Creating initial plan with ID: plan_1743084798
2025-03-27 22:13:22.388 | INFO     | app.llm:update_token_count:250 - Token usage: Input=353, Completion=160, Cumulative Input=353, Cumulative Completion=160, Total=513, Cumulative Total=513
2025-03-27 22:13:22.388 | INFO     | app.flow.planning:_create_initial_plan:181 - Plan creation result: Plan created successfully with ID: plan_1743084798

Plan: 七天日本旅游计划 (ID: plan_1743084798)
=====================================

Progress: 0/13 steps completed (0.0%)
Status: 0 completed, 0 in progress, 0 blocked, 13 not started

Steps:
0. [ ] 预订从深圳到日本的往返机票
1. [ ] 办理日本旅游签证
2. [ ] 预订日本的住宿（酒店或民宿）
3. [ ] 制定每日行程，包括旅游景点和活动
4. [ ] 研究日本的美食，并列出想尝试的餐厅
5. [ ] 准备旅行所需的物品和衣物
6. [ ] 出发前往深圳机场，飞往日本
7. [ ] 抵达日本，办理入境手续
8. [ ] 前往住宿地点，办理入住
9. [ ] 开始每日行程，游览景点和品尝美食
10. [ ] 注意当地的交通规则和文化习俗
11. [ ] 购买纪念品和礼物
12. [ ] 返回深圳，结束旅行

2025-03-27 22:13:22.389 | INFO     | app.logger:log_pretty_dict:53 - [app.agent.flow.PlanningFlow.execute] self.current_step_index: 0 step_info: :
{
  "text": "预订从深圳到日本的往返机票"
}
2025-03-27 22:13:22.389 | INFO     | app.flow.planning:_execute_step:265 - [app.agent.flow.PlanningFlow._execute_step] self._get_plan_text() -> plan_status Plan: 七天日本旅游计划 (ID: plan_1743084798)
=====================================

Progress: 0/13 steps completed (0.0%)
Status: 0 completed, 1 in progress, 0 blocked, 12 not started

Steps:
0. [→] 预订从深圳到日本的往返机票
1. [ ] 办理日本旅游签证
2. [ ] 预订日本的住宿（酒店或民宿）
3. [ ] 制定每日行程，包括旅游景点和活动
4. [ ] 研究日本的美食，并列出想尝试的餐厅
5. [ ] 准备旅行所需的物品和衣物
6. [ ] 出发前往深圳机场，飞往日本
7. [ ] 抵达日本，办理入境手续
8. [ ] 前往住宿地点，办理入住
9. [ ] 开始每日行程，游览景点和品尝美食
10. [ ] 注意当地的交通规则和文化习俗
11. [ ] 购买纪念品和礼物
12. [ ] 返回深圳，结束旅行

2025-03-27 22:13:22.390 | INFO     | app.agent.base:run:140 - Executing step 1/20
2025-03-27 22:13:23.548 | INFO     | app.llm:update_token_count:250 - Token usage: Input=1862, Completion=27, Cumulative Input=2215, Cumulative Completion=187, Total=1889, Cumulative Total=2402
2025-03-27 22:13:23.550 | INFO     | app.logger:log_pretty_dict:51 - [app.agent.ToolCallAgent][think][self.llm_ask_tool -> response] (原始值): ChatCompletionMessage(content=None, refusal=None, role='assistant', annotations=[], audio=None, function_call=None, tool_calls=[ChatCompletionMessageToolCall(id='call_eloigjXsbN45sylQPYYpEvmv', function=Function(arguments='{"action":"web_search","query":"book round trip flight from Shenzhen to Japan"}', name='browser_use'), type='function')])
2025-03-27 22:13:23.550 | INFO     | app.agent.toolcall:think:84 - ✨ Manus's thoughts: 
2025-03-27 22:13:23.550 | INFO     | app.agent.toolcall:think:85 - 🛠️ Manus selected 1 tools to use
2025-03-27 22:13:23.550 | INFO     | app.agent.toolcall:think:89 - 🧰 Tools being prepared: ['browser_use']
2025-03-27 22:13:23.550 | INFO     | app.agent.toolcall:think:92 - 🔧 Tool arguments: {"action":"web_search","query":"book round trip flight from Shenzhen to Japan"}
2025-03-27 22:13:23.553 | INFO     | app.agent.toolcall:execute_tool:184 - 🔧 Activating tool: 'browser_use'...
2025-03-27 22:13:26.323 | INFO     | app.tool.web_search:_try_all_engines:104 - 🔎 Attempting search with Google...
2025-03-27 22:13:30.673 | INFO     | app.tool.tool_collection:execute:32 - [app.too.ToolCollection.execute tool browser_use result: Searched for 'book round trip flight from Shenzhen to Japan' and navigated to first result: https://www.google.com/travel/flights?num=12&safe=active&sca_esv=2dbc18be17d9f5b4&hl=en&ie=UTF-8&source=flun&uitype=cuAR&gl=hk&curr=HKD&tfs=CAEQAhooEgoyMDI1LTA0LTEyagwIAhIIL20vMGxibXZyDAgCEggvbS8wN2RmaxooEgoyMDI1LTA0LTE2agwIAhIIL20vMDdkZmtyDAgCEggvbS8wbGJtdnpkQ2pSSVUwSjRaRGRKVmxRMlNVMUJRblZoTjJkQ1J5MHRMUzB0TFMwdExTMTJkR294T1VGQlFVRkJSMlpzV0ZGalEwRkxUbUZCRWdFd0dnb0kvdzhRQUJvRFNFdEVPREJ3MU0wQrIBBBgBIAE&ved=1t:3293&ictx=111
All results:https://www.google.com/travel/flights?num=12&safe=active&sca_esv=2dbc18be17d9f5b4&hl=en&ie=UTF-8&source=flun&uitype=cuAR&gl=hk&curr=HKD&tfs=CAEQAhooEgoyMDI1LTA0LTEyagwIAhIIL20vMGxibXZyDAgCEggvbS8wN2RmaxooEgoyMDI1LTA0LTE2agwIAhIIL20vMDdkZmtyDAgCEggvbS8wbGJtdnpkQ2pSSVUwSjRaRGRKVmxRMlNVMUJRblZoTjJkQ1J5MHRMUzB0TFMwdExTMTJkR294T1VGQlFVRkJSMlpzV0ZGalEwRkxUbUZCRWdFd0dnb0kvdzhRQUJvRFNFdEVPREJ3MU0wQrIBBBgBIAE&ved=1t:3293&ictx=111
https://www.expedia.com.hk/en/lp/flights/szx/nrt/shenzhen-to-tokyo
https://www.google.com/travel/flights/flights-from-shenzhen-to-tokyo.html
https://www.trip.com/flights/shenzhen-to-tokyo/airfares-szx-tyo/
https://www.skyscanner.com.hk/hk/en-gb/hkd/routes/szx/jp/shenzhen-bao-an-international-to-japan.html
https://www.expedia.com/lp/flights/szx/nrt/shenzhen-to-tokyo
https://www.skyscanner.com.hk/hk/en-gb/hkd/routes/szx/nrt/shenzhen-bao-an-international-to-tokyo-narita.html
https://www.google.com/travel/flights/flights-from-shenzhen-to-osaka.html
https://www.skyscanner.net/routes/szx/tyoa/shenzhen-bao-an-international-to-tokyo.html
https://www.kayak.com/flight-routes/Shenzhen-SZX/Tokyo-Narita-NRT]
2025-03-27 22:13:30.674 | INFO     | app.agent.toolcall:act:153 - 🎯 Tool 'browser_use' completed its mission! Result: Observed output of cmd `browser_use` executed:
Searched for 'book round trip flight from Shenzhen to Japan' and navigated to first result: https://www.google.com/travel/flights?num=12&safe=active&sca_esv=2dbc18be17d9f5b4&hl=en&ie=UTF-8&source=flun&uitype=cuAR&gl=hk&curr=HKD&tfs=CAEQAhooEgoyMDI1LTA0LTEyagwIAhIIL20vMGxibXZyDAgCEggvbS8wN2RmaxooEgoyMDI1LTA0LTE2agwIAhIIL20vMDdkZmtyDAgCEggvbS8wbGJtdnpkQ2pSSVUwSjRaRGRKVmxRMlNVMUJRblZoTjJkQ1J5MHRMUzB0TFMwdExTMTJkR294T1VGQlFVRkJSMlpzV0ZGalEwRkxUbUZCRWdFd0dnb0kvdzhRQUJvRFNFdEVPREJ3MU0wQrIBBBgBIAE&ved=1t:3293&ictx=111
All results:https://www.google.com/travel/flights?num=12&safe=active&sca_esv=2dbc18be17d9f5b4&hl=en&ie=UTF-8&source=flun&uitype=cuAR&gl=hk&curr=HKD&tfs=CAEQAhooEgoyMDI1LTA0LTEyagwIAhIIL20vMGxibXZyDAgCEggvbS8wN2RmaxooEgoyMDI1LTA0LTE2agwIAhIIL20vMDdkZmtyDAgCEggvbS8wbGJtdnpkQ2pSSVUwSjRaRGRKVmxRMlNVMUJRblZoTjJkQ1J5MHRMUzB0TFMwdExTMTJkR294T1VGQlFVRkJSMlpzV0ZGalEwRkxUbUZCRWdFd0dnb0kvdzhRQUJvRFNFdEVPREJ3MU0wQrIBBBgBIAE&ved=1t:3293&ictx=111
https://www.expedia.com.hk/en/lp/flights/szx/nrt/shenzhen-to-tokyo
https://www.google.com/travel/flights/flights-from-shenzhen-to-tokyo.html
https://www.trip.com/flights/shenzhen-to-tokyo/airfares-szx-tyo/
https://www.skyscanner.com.hk/hk/en-gb/hkd/routes/szx/jp/shenzhen-bao-an-international-to-japan.html
https://www.expedia.com/lp/flights/szx/nrt/shenzhen-to-tokyo
https://www.skyscanner.com.hk/hk/en-gb/hkd/routes/szx/nrt/shenzhen-bao-an-international-to-tokyo-narita.html
https://www.google.com/travel/flights/flights-from-shenzhen-to-osaka.html
https://www.skyscanner.net/routes/szx/tyoa/shenzhen-bao-an-international-to-tokyo.html
https://www.kayak.com/flight-routes/Shenzhen-SZX/Tokyo-Narita-NRT
2025-03-27 22:13:30.674 | INFO     | app.agent.toolcall:act:166 - [app.agent.ToolCallAgent.act -> results]: Observed output of cmd `browser_use` executed:
Searched for 'book round trip flight from Shenzhen to Japan' and navigated to first result: https://www.google.com/travel/flights?num=12&safe=active&sca_esv=2dbc18be17d9f5b4&hl=en&ie=UTF-8&source=flun&uitype=cuAR&gl=hk&curr=HKD&tfs=CAEQAhooEgoyMDI1LTA0LTEyagwIAhIIL20vMGxibXZyDAgCEggvbS8wN2RmaxooEgoyMDI1LTA0LTE2agwIAhIIL20vMDdkZmtyDAgCEggvbS8wbGJtdnpkQ2pSSVUwSjRaRGRKVmxRMlNVMUJRblZoTjJkQ1J5MHRMUzB0TFMwdExTMTJkR294T1VGQlFVRkJSMlpzV0ZGalEwRkxUbUZCRWdFd0dnb0kvdzhRQUJvRFNFdEVPREJ3MU0wQrIBBBgBIAE&ved=1t:3293&ictx=111
All results:https://www.google.com/travel/flights?num=12&safe=active&sca_esv=2dbc18be17d9f5b4&hl=en&ie=UTF-8&source=flun&uitype=cuAR&gl=hk&curr=HKD&tfs=CAEQAhooEgoyMDI1LTA0LTEyagwIAhIIL20vMGxibXZyDAgCEggvbS8wN2RmaxooEgoyMDI1LTA0LTE2agwIAhIIL20vMDdkZmtyDAgCEggvbS8wbGJtdnpkQ2pSSVUwSjRaRGRKVmxRMlNVMUJRblZoTjJkQ1J5MHRMUzB0TFMwdExTMTJkR294T1VGQlFVRkJSMlpzV0ZGalEwRkxUbUZCRWdFd0dnb0kvdzhRQUJvRFNFdEVPREJ3MU0wQrIBBBgBIAE&ved=1t:3293&ictx=111
https://www.expedia.com.hk/en/lp/flights/szx/nrt/shenzhen-to-tokyo
https://www.google.com/travel/flights/flights-from-shenzhen-to-tokyo.html
https://www.trip.com/flights/shenzhen-to-tokyo/airfares-szx-tyo/
https://www.skyscanner.com.hk/hk/en-gb/hkd/routes/szx/jp/shenzhen-bao-an-international-to-japan.html
https://www.expedia.com/lp/flights/szx/nrt/shenzhen-to-tokyo
https://www.skyscanner.com.hk/hk/en-gb/hkd/routes/szx/nrt/shenzhen-bao-an-international-to-tokyo-narita.html
https://www.google.com/travel/flights/flights-from-shenzhen-to-osaka.html
https://www.skyscanner.net/routes/szx/tyoa/shenzhen-bao-an-international-to-tokyo.html
https://www.kayak.com/flight-routes/Shenzhen-SZX/Tokyo-Narita-NRT
2025-03-27 22:13:30.675 | INFO     | app.logger:log_pretty_dict:53 - [app.agent.BaseAgent][run self.step()-> step_result]:
"Observed output of cmd `browser_use` executed:\nSearched for 'book round trip flight from Shenzhen to Japan' and navigated to first result: https://www.google.com/travel/flights?num=12&safe=active&sca_esv=2dbc18be17d9f5b4&hl=en&ie=UTF-8&source=flun&uitype=cuAR&gl=hk&curr=HKD&tfs=CAEQAhooEgoyMDI1LTA0LTEyagwIAhIIL20vMGxibXZyDAgCEggvbS8wN2RmaxooEgoyMDI1LTA0LTE2agwIAhIIL20vMDdkZmtyDAgCEggvbS8wbGJtdnpkQ2pSSVUwSjRaRGRKVmxRMlNVMUJRblZoTjJkQ1J5MHRMUzB0TFMwdExTMTJkR294T1VGQlFVRkJSMlpzV0ZGalEwRkxUbUZCRWdFd0dnb0kvdzhRQUJvRFNFdEVPREJ3MU0wQrIBBBgBIAE&ved=1t:3293&ictx=111\nAll results:https://www.google.com/travel/flights?num=12&safe=active&sca_esv=2dbc18be17d9f5b4&hl=en&ie=UTF-8&source=flun&uitype=cuAR&gl=hk&curr=HKD&tfs=CAEQAhooEgoyMDI1LTA0LTEyagwIAhIIL20vMGxibXZyDAgCEggvbS8wN2RmaxooEgoyMDI1LTA0LTE2agwIAhIIL20vMDdkZmtyDAgCEggvbS8wbGJtdnpkQ2pSSVUwSjRaRGRKVmxRMlNVMUJRblZoTjJkQ1J5MHRMUzB0TFMwdExTMTJkR294T1VGQlFVRkJSMlpzV0ZGalEwRkxUbUZCRWdFd0dnb0kvdzhRQUJvRFNFdEVPREJ3MU0wQrIBBBgBIAE&ved=1t:3293&ictx=111\nhttps://www.expedia.com.hk/en/lp/flights/szx/nrt/shenzhen-to-tokyo\nhttps://www.google.com/travel/flights/flights-from-shenzhen-to-tokyo.html\nhttps://www.trip.com/flights/shenzhen-to-tokyo/airfares-szx-tyo/\nhttps://www.skyscanner.com.hk/hk/en-gb/hkd/routes/szx/jp/shenzhen-bao-an-international-to-japan.html\nhttps://www.expedia.com/lp/flights/szx/nrt/shenzhen-to-tokyo\nhttps://www.skyscanner.com.hk/hk/en-gb/hkd/routes/szx/nrt/shenzhen-bao-an-international-to-tokyo-narita.html\nhttps://www.google.com/travel/flights/flights-from-shenzhen-to-osaka.html\nhttps://www.skyscanner.net/routes/szx/tyoa/shenzhen-bao-an-international-to-tokyo.html\nhttps://www.kayak.com/flight-routes/Shenzhen-SZX/Tokyo-Narita-NRT"
2025-03-27 22:13:30.675 | INFO     | app.agent.base:run:140 - Executing step 2/20
2025-03-27 22:13:38.751 | INFO     | app.llm:update_token_count:250 - Token usage: Input=3866, Completion=20, Cumulative Input=6081, Cumulative Completion=207, Total=3886, Cumulative Total=6288
2025-03-27 22:13:38.752 | INFO     | app.logger:log_pretty_dict:51 - [app.agent.ToolCallAgent][think][self.llm_ask_tool -> response] (原始值): ChatCompletionMessage(content=None, refusal=None, role='assistant', annotations=[], audio=None, function_call=None, tool_calls=[ChatCompletionMessageToolCall(id='call_ZTfy3rEAt0VVUMfa1cbq1WqE', function=Function(arguments='{"action":"click_element","index":43}', name='browser_use'), type='function')])
2025-03-27 22:13:38.752 | INFO     | app.agent.toolcall:think:84 - ✨ Manus's thoughts: 
2025-03-27 22:13:38.752 | INFO     | app.agent.toolcall:think:85 - 🛠️ Manus selected 1 tools to use
2025-03-27 22:13:38.752 | INFO     | app.agent.toolcall:think:89 - 🧰 Tools being prepared: ['browser_use']
2025-03-27 22:13:38.752 | INFO     | app.agent.toolcall:think:92 - 🔧 Tool arguments: {"action":"click_element","index":43}
2025-03-27 22:13:38.753 | INFO     | app.agent.toolcall:execute_tool:184 - 🔧 Activating tool: 'browser_use'...
2025-03-27 22:13:38.767 | INFO     | app.tool.tool_collection:execute:32 - [app.too.ToolCollection.execute tool browser_use result: Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:43, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:43, in-viewport] not found]
2025-03-27 22:13:38.767 | INFO     | app.agent.toolcall:act:153 - 🎯 Tool 'browser_use' completed its mission! Result: Observed output of cmd `browser_use` executed:
Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:43, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:43, in-viewport] not found
2025-03-27 22:13:38.767 | INFO     | app.agent.toolcall:act:166 - [app.agent.ToolCallAgent.act -> results]: Observed output of cmd `browser_use` executed:
Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:43, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:43, in-viewport] not found
2025-03-27 22:13:38.768 | INFO     | app.logger:log_pretty_dict:53 - [app.agent.BaseAgent][run self.step()-> step_result]:
"Observed output of cmd `browser_use` executed:\nError: Browser action 'click_element' failed: Failed to click element: <button class=\"VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jscontroller=\"soHxf\" jsaction=\"click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;\" data-idom-class=\"nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jsname=\"LgbsSe\" aria-label=\"Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12.\" aria-expanded=\"false\"> [interactive, top, highlight:43, in-viewport]. Error: Element: <button class=\"VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jscontroller=\"soHxf\" jsaction=\"click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;\" data-idom-class=\"nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jsname=\"LgbsSe\" aria-label=\"Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12.\" aria-expanded=\"false\"> [interactive, top, highlight:43, in-viewport] not found"
2025-03-27 22:13:38.768 | INFO     | app.agent.base:run:140 - Executing step 3/20
2025-03-27 22:13:46.640 | INFO     | app.llm:update_token_count:250 - Token usage: Input=5775, Completion=20, Cumulative Input=11856, Cumulative Completion=227, Total=5795, Cumulative Total=12083
2025-03-27 22:13:46.642 | INFO     | app.logger:log_pretty_dict:51 - [app.agent.ToolCallAgent][think][self.llm_ask_tool -> response] (原始值): ChatCompletionMessage(content=None, refusal=None, role='assistant', annotations=[], audio=None, function_call=None, tool_calls=[ChatCompletionMessageToolCall(id='call_c5fzJOM67EYu94TwpreofUWB', function=Function(arguments='{"action":"click_element","index":45}', name='browser_use'), type='function')])
2025-03-27 22:13:46.642 | INFO     | app.agent.toolcall:think:84 - ✨ Manus's thoughts: 
2025-03-27 22:13:46.642 | INFO     | app.agent.toolcall:think:85 - 🛠️ Manus selected 1 tools to use
2025-03-27 22:13:46.642 | INFO     | app.agent.toolcall:think:89 - 🧰 Tools being prepared: ['browser_use']
2025-03-27 22:13:46.642 | INFO     | app.agent.toolcall:think:92 - 🔧 Tool arguments: {"action":"click_element","index":45}
2025-03-27 22:13:46.643 | INFO     | app.agent.toolcall:execute_tool:184 - 🔧 Activating tool: 'browser_use'...
2025-03-27 22:13:46.652 | INFO     | app.tool.tool_collection:execute:32 - [app.too.ToolCollection.execute tool browser_use result: Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 12:25 PM on Saturday, April 12 and arrives at Narita International Airport at 6:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:45, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 12:25 PM on Saturday, April 12 and arrives at Narita International Airport at 6:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:45, in-viewport] not found]
2025-03-27 22:13:46.652 | INFO     | app.agent.toolcall:act:153 - 🎯 Tool 'browser_use' completed its mission! Result: Observed output of cmd `browser_use` executed:
Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 12:25 PM on Saturday, April 12 and arrives at Narita International Airport at 6:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:45, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 12:25 PM on Saturday, April 12 and arrives at Narita International Airport at 6:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:45, in-viewport] not found
2025-03-27 22:13:46.653 | INFO     | app.agent.toolcall:act:166 - [app.agent.ToolCallAgent.act -> results]: Observed output of cmd `browser_use` executed:
Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 12:25 PM on Saturday, April 12 and arrives at Narita International Airport at 6:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:45, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 12:25 PM on Saturday, April 12 and arrives at Narita International Airport at 6:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:45, in-viewport] not found
2025-03-27 22:13:46.653 | INFO     | app.logger:log_pretty_dict:53 - [app.agent.BaseAgent][run self.step()-> step_result]:
"Observed output of cmd `browser_use` executed:\nError: Browser action 'click_element' failed: Failed to click element: <button class=\"VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jscontroller=\"soHxf\" jsaction=\"click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;\" data-idom-class=\"nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jsname=\"LgbsSe\" aria-label=\"Flight details. Leaves Shenzhen Bao'an International Airport at 12:25 PM on Saturday, April 12 and arrives at Narita International Airport at 6:00 PM on Saturday, April 12.\" aria-expanded=\"false\"> [interactive, top, highlight:45, in-viewport]. Error: Element: <button class=\"VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jscontroller=\"soHxf\" jsaction=\"click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;\" data-idom-class=\"nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jsname=\"LgbsSe\" aria-label=\"Flight details. Leaves Shenzhen Bao'an International Airport at 12:25 PM on Saturday, April 12 and arrives at Narita International Airport at 6:00 PM on Saturday, April 12.\" aria-expanded=\"false\"> [interactive, top, highlight:45, in-viewport] not found"
2025-03-27 22:13:46.653 | INFO     | app.agent.base:run:140 - Executing step 4/20
2025-03-27 22:13:55.856 | INFO     | app.llm:update_token_count:250 - Token usage: Input=7684, Completion=20, Cumulative Input=19540, Cumulative Completion=247, Total=7704, Cumulative Total=19787
2025-03-27 22:13:55.857 | INFO     | app.logger:log_pretty_dict:51 - [app.agent.ToolCallAgent][think][self.llm_ask_tool -> response] (原始值): ChatCompletionMessage(content=None, refusal=None, role='assistant', annotations=[], audio=None, function_call=None, tool_calls=[ChatCompletionMessageToolCall(id='call_XStGyEkTw0RdeRvBn2jBAqFj', function=Function(arguments='{"action":"click_element","index":46}', name='browser_use'), type='function')])
2025-03-27 22:13:55.857 | INFO     | app.agent.toolcall:think:84 - ✨ Manus's thoughts: 
2025-03-27 22:13:55.857 | INFO     | app.agent.toolcall:think:85 - 🛠️ Manus selected 1 tools to use
2025-03-27 22:13:55.858 | INFO     | app.agent.toolcall:think:89 - 🧰 Tools being prepared: ['browser_use']
2025-03-27 22:13:55.858 | INFO     | app.agent.toolcall:think:92 - 🔧 Tool arguments: {"action":"click_element","index":46}
2025-03-27 22:13:55.858 | INFO     | app.agent.toolcall:execute_tool:184 - 🔧 Activating tool: 'browser_use'...
2025-03-27 22:13:55.866 | INFO     | app.tool.tool_collection:execute:32 - [app.too.ToolCollection.execute tool browser_use result: Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 5:00 PM on Saturday, April 12 and arrives at Haneda Airport at 10:20 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:46, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 5:00 PM on Saturday, April 12 and arrives at Haneda Airport at 10:20 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:46, in-viewport] not found]
2025-03-27 22:13:55.866 | INFO     | app.agent.toolcall:act:153 - 🎯 Tool 'browser_use' completed its mission! Result: Observed output of cmd `browser_use` executed:
Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 5:00 PM on Saturday, April 12 and arrives at Haneda Airport at 10:20 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:46, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 5:00 PM on Saturday, April 12 and arrives at Haneda Airport at 10:20 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:46, in-viewport] not found
2025-03-27 22:13:55.867 | INFO     | app.agent.toolcall:act:166 - [app.agent.ToolCallAgent.act -> results]: Observed output of cmd `browser_use` executed:
Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 5:00 PM on Saturday, April 12 and arrives at Haneda Airport at 10:20 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:46, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 5:00 PM on Saturday, April 12 and arrives at Haneda Airport at 10:20 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:46, in-viewport] not found
2025-03-27 22:13:55.867 | INFO     | app.logger:log_pretty_dict:53 - [app.agent.BaseAgent][run self.step()-> step_result]:
"Observed output of cmd `browser_use` executed:\nError: Browser action 'click_element' failed: Failed to click element: <button class=\"VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jscontroller=\"soHxf\" jsaction=\"click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd\" data-idom-class=\"nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jsname=\"LgbsSe\" aria-label=\"Flight details. Leaves Shenzhen Bao'an International Airport at 5:00 PM on Saturday, April 12 and arrives at Haneda Airport at 10:20 PM on Saturday, April 12.\" aria-expanded=\"false\"> [interactive, top, highlight:46, in-viewport]. Error: Element: <button class=\"VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jscontroller=\"soHxf\" jsaction=\"click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd\" data-idom-class=\"nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jsname=\"LgbsSe\" aria-label=\"Flight details. Leaves Shenzhen Bao'an International Airport at 5:00 PM on Saturday, April 12 and arrives at Haneda Airport at 10:20 PM on Saturday, April 12.\" aria-expanded=\"false\"> [interactive, top, highlight:46, in-viewport] not found"
2025-03-27 22:13:55.867 | INFO     | app.agent.base:run:140 - Executing step 5/20
2025-03-27 22:14:06.912 | INFO     | app.llm:update_token_count:250 - Token usage: Input=9591, Completion=20, Cumulative Input=29131, Cumulative Completion=267, Total=9611, Cumulative Total=29398
2025-03-27 22:14:06.914 | INFO     | app.logger:log_pretty_dict:51 - [app.agent.ToolCallAgent][think][self.llm_ask_tool -> response] (原始值): ChatCompletionMessage(content=None, refusal=None, role='assistant', annotations=[], audio=None, function_call=None, tool_calls=[ChatCompletionMessageToolCall(id='call_kzbcKCvGcJVHbX7EVLhTek9P', function=Function(arguments='{"action":"click_element","index":42}', name='browser_use'), type='function')])
2025-03-27 22:14:06.914 | INFO     | app.agent.toolcall:think:84 - ✨ Manus's thoughts: 
2025-03-27 22:14:06.914 | INFO     | app.agent.toolcall:think:85 - 🛠️ Manus selected 1 tools to use
2025-03-27 22:14:06.914 | INFO     | app.agent.toolcall:think:89 - 🧰 Tools being prepared: ['browser_use']
2025-03-27 22:14:06.914 | INFO     | app.agent.toolcall:think:92 - 🔧 Tool arguments: {"action":"click_element","index":42}
2025-03-27 22:14:06.915 | INFO     | app.agent.toolcall:execute_tool:184 - 🔧 Activating tool: 'browser_use'...
2025-03-27 22:14:06.923 | INFO     | app.tool.tool_collection:execute:32 - [app.too.ToolCollection.execute tool browser_use result: Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:42, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:42, in-viewport] not found]
2025-03-27 22:14:06.924 | INFO     | app.agent.toolcall:act:153 - 🎯 Tool 'browser_use' completed its mission! Result: Observed output of cmd `browser_use` executed:
Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:42, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:42, in-viewport] not found
2025-03-27 22:14:06.924 | INFO     | app.agent.toolcall:act:166 - [app.agent.ToolCallAgent.act -> results]: Observed output of cmd `browser_use` executed:
Error: Browser action 'click_element' failed: Failed to click element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:42, in-viewport]. Error: Element: <button class="VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jscontroller="soHxf" jsaction="click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;" data-idom-class="nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc" jsname="LgbsSe" aria-label="Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12." aria-expanded="false"> [interactive, top, highlight:42, in-viewport] not found
2025-03-27 22:14:06.925 | INFO     | app.logger:log_pretty_dict:53 - [app.agent.BaseAgent][run self.step()-> step_result]:
"Observed output of cmd `browser_use` executed:\nError: Browser action 'click_element' failed: Failed to click element: <button class=\"VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jscontroller=\"soHxf\" jsaction=\"click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;\" data-idom-class=\"nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jsname=\"LgbsSe\" aria-label=\"Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12.\" aria-expanded=\"false\"> [interactive, top, highlight:42, in-viewport]. Error: Element: <button class=\"VfPpkd-LgbsSe VfPpkd-LgbsSe-OWXEXe-k8QpJ VfPpkd-LgbsSe-OWXEXe-Bz112c-M1Soyc VfPpkd-LgbsSe-OWXEXe-dgl2Hf nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jscontroller=\"soHxf\" jsaction=\"click:cOuCgd; mousedown:UX7yZ; mouseup:lbsD7e; mouseenter:tfO1Yc; mouseleave:JywGue; touchstart:p6p2H; touchmove:FwuNnf; touchend:yfqBxc; touchcancel:JMtRjd; focus:AHmuwe; blur:O22p3e; contextmenu:mg9Pef;mlnRJb:fLiPzd;\" data-idom-class=\"nCP5yc AjY5Oe LQeN7 nJawce OTelKf XPGpHc mAozAc\" jsname=\"LgbsSe\" aria-label=\"Flight details. Leaves Shenzhen Bao'an International Airport at 9:45 AM on Saturday, April 12 and arrives at Narita International Airport at 3:00 PM on Saturday, April 12.\" aria-expanded=\"false\"> [interactive, top, highlight:42, in-viewport] not found"
2025-03-27 22:14:06.925 | INFO     | app.agent.base:run:140 - Executing step 6/20
2025-03-27 22:14:40.908 | INFO     | app.llm:update_token_count:250 - Token usage: Input=11500, Completion=20, Cumulative Input=40631, Cumulative Completion=287, Total=11520, Cumulative Total=40918
2025-03-27 22:14:40.909 | INFO     | app.logger:log_pretty_dict:51 - [app.agent.ToolCallAgent][think][self.llm_ask_tool -> response] (原始值): ChatCompletionMessage(content=None, refusal=None, role='assistant', annotations=[], audio=None, function_call=None, tool_calls=[ChatCompletionMessageToolCall(id='call_NUmkRjkfQsiKN3iQIae3yReY', function=Function(arguments='{"action":"click_element","index":64}', name='browser_use'), type='function')])
2025-03-27 22:14:40.909 | INFO     | app.agent.toolcall:think:84 - ✨ Manus's thoughts: 
2025-03-27 22:14:40.909 | INFO     | app.agent.toolcall:think:85 - 🛠️ Manus selected 1 tools to use
2025-03-27 22:14:40.909 | INFO     | app.agent.toolcall:think:89 - 🧰 Tools being prepared: ['browser_use']
2025-03-27 22:14:40.909 | INFO     | app.agent.toolcall:think:92 - 🔧 Tool arguments: {"action":"click_element","index":64}
2025-03-27 22:14:40.909 | INFO     | app.agent.toolcall:execute_tool:184 - 🔧 Activating tool: 'browser_use'...
2025-03-27 22:14:41.201 | INFO     | app.tool.tool_collection:execute:32 - [app.too.ToolCollection.execute tool browser_use result: Clicked element at index 64]
2025-03-27 22:14:41.202 | INFO     | app.agent.toolcall:act:153 - 🎯 Tool 'browser_use' completed its mission! Result: Observed output of cmd `browser_use` executed:
Clicked element at index 64
2025-03-27 22:14:41.202 | INFO     | app.agent.toolcall:act:166 - [app.agent.ToolCallAgent.act -> results]: Observed output of cmd `browser_use` executed:
Clicked element at index 64
2025-03-27 22:14:41.202 | INFO     | app.logger:log_pretty_dict:53 - [app.agent.BaseAgent][run self.step()-> step_result]:
"Observed output of cmd `browser_use` executed:\nClicked element at index 64"
2025-03-27 22:14:41.202 | INFO     | app.agent.base:run:140 - Executing step 7/20
2025-03-27 22:15:38.557 | ERROR    | app.llm:ask_tool:766 - OpenAI API error: Connection error.
2025-03-27 22:15:38.557 | ERROR    | app.llm:ask_tool:772 - API error: Connection error.
2025-03-27 22:16:32.387 | ERROR    | app.llm:ask_tool:766 - OpenAI API error: Connection error.
2025-03-27 22:16:32.387 | ERROR    | app.llm:ask_tool:772 - API error: Connection error.
2025-03-27 22:17:27.257 | ERROR    | app.llm:ask_tool:766 - OpenAI API error: Connection error.
2025-03-27 22:17:27.258 | ERROR    | app.llm:ask_tool:772 - API error: Connection error.
2025-03-27 22:18:22.314 | INFO     | app.llm:update_token_count:250 - Token usage: Input=12637, Completion=20, Cumulative Input=53268, Cumulative Completion=307, Total=12657, Cumulative Total=53575
2025-03-27 22:18:22.315 | INFO     | app.logger:log_pretty_dict:51 - [app.agent.ToolCallAgent][think][self.llm_ask_tool -> response] (原始值): ChatCompletionMessage(content=None, refusal=None, role='assistant', annotations=[], audio=None, function_call=None, tool_calls=[ChatCompletionMessageToolCall(id='call_U4GKqTGDvc5nkYbq8UvJ2zSP', function=Function(arguments='{"action":"click_element","index":42}', name='browser_use'), type='function')])
2025-03-27 22:18:22.316 | INFO     | app.agent.toolcall:think:84 - ✨ Manus's thoughts: 
2025-03-27 22:18:22.316 | INFO     | app.agent.toolcall:think:85 - 🛠️ Manus selected 1 tools to use
2025-03-27 22:18:22.316 | INFO     | app.agent.toolcall:think:89 - 🧰 Tools being prepared: ['browser_use']
2025-03-27 22:18:22.316 | INFO     | app.agent.toolcall:think:92 - 🔧 Tool arguments: {"action":"click_element","index":42}
2025-03-27 22:18:22.316 | INFO     | app.agent.toolcall:execute_tool:184 - 🔧 Activating tool: 'browser_use'...
2025-03-27 22:18:22.551 | INFO     | app.tool.tool_collection:execute:32 - [app.too.ToolCollection.execute tool browser_use result: Clicked element at index 42]
2025-03-27 22:18:22.552 | INFO     | app.agent.toolcall:act:153 - 🎯 Tool 'browser_use' completed its mission! Result: Observed output of cmd `browser_use` executed:
Clicked element at index 42
2025-03-27 22:18:22.552 | INFO     | app.agent.toolcall:act:166 - [app.agent.ToolCallAgent.act -> results]: Observed output of cmd `browser_use` executed:
Clicked element at index 42
2025-03-27 22:18:22.552 | INFO     | app.logger:log_pretty_dict:53 - [app.agent.BaseAgent][run self.step()-> step_result]:
"Observed output of cmd `browser_use` executed:\nClicked element at index 42"
2025-03-27 22:18:22.552 | INFO     | app.agent.base:run:140 - Executing step 8/20
2025-03-27 22:19:27.065 | ERROR    | app.llm:ask_tool:766 - OpenAI API error: Connection error.
2025-03-27 22:19:27.066 | ERROR    | app.llm:ask_tool:772 - API error: Connection error.
2025-03-27 22:20:29.297 | ERROR    | app.llm:ask_tool:766 - OpenAI API error: Connection error.
2025-03-27 22:20:29.297 | ERROR    | app.llm:ask_tool:772 - API error: Connection error.
^CTraceback (most recent call last):
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/asyncio/runners.py", line 118, in run
    return self._loop.run_until_complete(task)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/asyncio/base_events.py", line 691, in run_until_complete
    return future.result()
           ^^^^^^^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/run_flow.py", line 29, in run_flow
    result = await asyncio.wait_for(
             ^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/asyncio/tasks.py", line 520, in wait_for
    return await fut
           ^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/app/flow/planning.py", line 125, in execute
    step_result = await self._execute_step(executor, step_info)
                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/app/flow/planning.py", line 281, in _execute_step
    step_result = await executor.run(step_prompt)
                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/app/agent/base.py", line 141, in run
    step_result = await self.step()
                  ^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/app/agent/react.py", line 35, in step
    should_act = await self.think()
                 ^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/app/agent/manus.py", line 58, in think
    result = await super().think()
             ^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/app/agent/browser.py", line 124, in think
    result = await super().think()
             ^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/app/agent/toolcall.py", line 46, in think
    response = await self.llm.ask_tool(
               ^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/tenacity/asyncio/__init__.py", line 189, in async_wrapped
    return await copy(fn, *args, **kwargs)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/tenacity/asyncio/__init__.py", line 111, in __call__
    do = await self.iter(retry_state=retry_state)
         ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/tenacity/asyncio/__init__.py", line 153, in iter
    result = await action(retry_state)
             ^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/tenacity/_utils.py", line 99, in inner
    return call(*args, **kwargs)
           ^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/tenacity/__init__.py", line 398, in <lambda>
    self._add_action_func(lambda rs: rs.outcome.result())
                                     ^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/concurrent/futures/_base.py", line 449, in result
    return self.__get_result()
           ^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/concurrent/futures/_base.py", line 401, in __get_result
    raise self._exception
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/tenacity/asyncio/__init__.py", line 114, in __call__
    result = await fn(*args, **kwargs)
             ^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/Desktop/OpenManus/app/llm.py", line 740, in ask_tool
    response: ChatCompletion = await self.client.chat.completions.create(
                               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/openai/resources/chat/completions/completions.py", line 2000, in create
    return await self._post(
           ^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/openai/_base_client.py", line 1767, in post
    return await self.request(cast_to, opts, stream=stream, stream_cls=stream_cls)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/openai/_base_client.py", line 1461, in request
    return await self._request(
           ^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/openai/_base_client.py", line 1500, in _request
    response = await self._client.send(
               ^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpx/_client.py", line 1629, in send
    response = await self._send_handling_auth(
               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpx/_client.py", line 1657, in _send_handling_auth
    response = await self._send_handling_redirects(
               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpx/_client.py", line 1694, in _send_handling_redirects
    response = await self._send_single_request(request)
               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpx/_client.py", line 1730, in _send_single_request
    response = await transport.handle_async_request(request)
               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpx/_transports/default.py", line 394, in handle_async_request
    resp = await self._pool.handle_async_request(req)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpcore/_async/connection_pool.py", line 256, in handle_async_request
    raise exc from None
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpcore/_async/connection_pool.py", line 236, in handle_async_request
    response = await connection.handle_async_request(
               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpcore/_async/connection.py", line 103, in handle_async_request
    return await self._connection.handle_async_request(request)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpcore/_async/http11.py", line 136, in handle_async_request
    raise exc
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpcore/_async/http11.py", line 88, in handle_async_request
    await self._send_request_body(**kwargs)
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpcore/_async/http11.py", line 159, in _send_request_body
    await self._send_event(event, timeout=timeout)
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpcore/_async/http11.py", line 166, in _send_event
    await self._network_stream.write(bytes_to_send, timeout=timeout)
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/httpcore/_backends/anyio.py", line 50, in write
    await self._stream.send(item=buffer)
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/anyio/streams/tls.py", line 226, in send
    await self._call_sslobject_method(self._ssl_object.write, item)
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/anyio/streams/tls.py", line 192, in _call_sslobject_method
    await self.transport_stream.send(self._write_bio.read())
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/site-packages/anyio/_backends/_asyncio.py", line 1298, in send
    await self._protocol.write_event.wait()
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/asyncio/locks.py", line 212, in wait
    await fut
asyncio.exceptions.CancelledError

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/Users/tanbowen/Desktop/OpenManus/run_flow.py", line 49, in <module>
    asyncio.run(run_flow())
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/asyncio/runners.py", line 195, in run
    return runner.run(main)
           ^^^^^^^^^^^^^^^^
  File "/Users/tanbowen/micromamba/envs/open_manus/lib/python3.12/asyncio/runners.py", line 123, in run
    raise KeyboardInterrupt()
KeyboardInterrupt
