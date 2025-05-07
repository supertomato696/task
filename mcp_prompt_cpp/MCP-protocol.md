**You:**

最近 modul context protocol 很火热，我希望你主要基于我提供的两个mcp 网址对这个项目进行研究。如果涉及编程语言，只研究或者介绍和 python 相关的。 我希望深入了解mcp 协议，组成部分，核心架构等。如果本次不能介绍开发相关的，可以下次研究时在深入详细介绍开发相关的。https://modelcontextprotocol.io/quickstart/server 和https://github.com/modelcontextprotocol

* * *

**ChatGPT:**

好的，我会围绕 MCP（Model Context Protocol）展开研究。为了确保方向正确，请确认以下几点：

1. 你是否希望我本次的研究重点是：
    
    * MCP 协议的设计原理、组成结构、核心架构、工作机制，以及和 Python 语言的集成？
        
    * 不包括开发实现（即不深入代码和动手部分），但可以为后续开发研究打基础？
        
2. 是否只需要参考你提供的两个网址（官网和 GitHub），还是也欢迎我扩展查阅其他公开的社区内容、文档或讨论（如 Hacker News、Reddit、博客等）以补充资料？
    

确认这些之后，我就可以立刻开始研究。

* * *

**You:**

本次研究的重点MCP 协议的设计原理、组成结构、核心架构、工作机制，以及和 Python 语言的集成，可以扩展查阅其他公开的社区内容、文档或讨论（如 Hacker News、Reddit、博客等）以补充资料

* * *

**ChatGPT:**

明白了，我将研究 Model Context Protocol（MCP）的协议设计原理、核心架构、组成结构和工作机制，并重点关注其与 Python 的集成方式。

我会基于你提供的两个核心网址（官网和 GitHub）为主，同时查阅公开的社区内容（如论坛讨论、技术博客和开发者反馈）来补充资料。

整理完成后会向你详细汇报分析结果。

* * *

**ChatGPT:**

# 模型上下文协议（MCP）研究报告

![https://www.philschmid.de/mcp-introduction](blob:https://chatgpt.com/3e263699-b927-43a2-ba66-b08ba07903b9) _图0：引入MCP前后AI应用集成方式对比：在MCP出现之前，每个LLM应用需要针对每个数据源定制集成（左图，每条灰线表示特定API集成）；而采用MCP之后，LLM通过统一的MCP接口连接，可同时访问多种外部数据源（右图）[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=Think%20of%20it%20like%20USB,effort%20across%20teams%2C%20inconsistent%20implementations)。_

## 协议目标与背景

**模型上下文协议（Model Context Protocol, MCP）**由 Anthropic 于 2024 年开源推出，是一种开放标准协议，旨在为大型语言模型（LLM）应用提供标准化接口，以安全、统一的方式连接和交互各类外部数据源与工具[zh.wikipedia.org](https://zh.wikipedia.org/wiki/%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE#:~:text=%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE%EF%BC%88%E8%8B%B1%E8%AF%AD%EF%BC%9AModel%20Context%20Protocol%EF%BC%8C%E7%BC%A9%E5%86%99%EF%BC%9AMCP%EF%BC%89%E6%98%AF%20Anthropic%20,2%20%5D%E3%80%82)[anthropic.com](https://www.anthropic.com/news/model-context-protocol#:~:text=Today%2C%20we%27re%20open,produce%20better%2C%20more%20relevant%20responses)。其核心目标是解决当前 LLM 应用因“信息孤岛”造成的局限：传统上，每个模型应用需要针对每个外部系统开发专有集成，存在大量重复工作且难以扩展。这被形象地比喻为一个 M×N 的集成难题（M 个AI应用对接 N 个数据源）[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=Think%20of%20it%20like%20USB,effort%20across%20teams%2C%20inconsistent%20implementations)。MCP 提供了类似“AI 应用的 USB-C 接口”[cnblogs.com](https://www.cnblogs.com/shanren/p/18797313#:~:text=%E7%9B%AE%E6%A0%87%E6%98%AF%E6%88%90%E4%B8%BA%20AI%20%E9%A2%86%E5%9F%9F%E7%9A%84%E2%80%9CHTTP%20%E5%8D%8F%E8%AE%AE%E2%80%9D%EF%BC%8C%E6%8E%A8%E5%8A%A8%20LLM,%E5%BA%94%E7%94%A8%E7%9A%84%E6%A0%87%E5%87%86%E5%8C%96%E5%92%8C%E5%8E%BB%E4%B8%AD%E5%BF%83%E5%8C%96%E3%80%82)的通用协议，将集成复杂度从 M×N 转化为 M+N：外部工具或数据源的开发者只需各自实现一个 MCP **服务器**，而应用开发者只需为各自的应用实现一个 MCP **客户端**，双方通过统一协议对接[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=MCP%20aims%20to%20simplify%20this,server%20architecture%20where)。通过取代碎片化的定制集成方案，MCP 为 LLM 获取所需数据提供了一条简单可靠的通道[anthropic.com](https://www.anthropic.com/news/model-context-protocol#:~:text=MCP%20addresses%20this%20challenge,to%20the%20data%20they%20need)。总而言之，MCP 的设计初衷是在保持安全性的前提下，让 AI 模型突破自身训练数据的限制，动态访问实时的上下文信息和服务，从而生成更相关、更准确的响应[zh.wikipedia.org](https://zh.wikipedia.org/wiki/%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE#:~:text=%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE%EF%BC%88%E8%8B%B1%E8%AF%AD%EF%BC%9AModel%20Context%20Protocol%EF%BC%8C%E7%BC%A9%E5%86%99%EF%BC%9AMCP%EF%BC%89%E6%98%AF%20Anthropic%20,2%20%5D%E3%80%82)[zh.wikipedia.org](https://zh.wikipedia.org/wiki/%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE#:~:text=%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE%20,%E6%9C%8D%E5%8A%A1%E5%99%A8%E5%88%99%E8%B4%9F%E8%B4%A3%E4%B8%8E%E7%9B%B8%E5%BA%94%E7%9A%84%E5%A4%96%E9%83%A8%E6%95%B0%E6%8D%AE%E6%BA%90%E6%88%96%E5%B7%A5%E5%85%B7%E8%BF%9B%E8%A1%8C%E4%BA%A4%E4%BA%92%EF%BC%8C%E8%8E%B7%E5%8F%96%E6%95%B0%E6%8D%AE%E5%B9%B6%E6%8C%89%E7%85%A7%20MCP%20%E5%8D%8F%E8%AE%AE%E8%A7%84%E8%8C%83%E8%BF%9B%E8%A1%8C%E6%A0%BC%E5%BC%8F%E5%8C%96%EF%BC%8C%E6%9C%80%E5%90%8E%E5%B0%86%E6%A0%BC%E5%BC%8F%E5%8C%96%E5%90%8E%E7%9A%84%E6%95%B0%E6%8D%AE%E8%BF%94%E5%9B%9E%E7%BB%99%20LLM%E3%80%82LLM%20%E6%8E%A5%E6%94%B6%E5%88%B0%E4%B8%8A%E4%B8%8B%E6%96%87%E4%BF%A1%E6%81%AF%E5%90%8E%EF%BC%8C%E5%8F%AF%E4%BB%A5%E5%B0%86%E5%85%B6%E7%94%A8%E4%BA%8E%E7%94%9F%E6%88%90%E6%9B%B4%E5%87%86%E7%A1%AE%E3%80%81%E6%9B%B4%E7%9B%B8%E5%85%B3%E7%9A%84%E5%9B%9E%E5%A4%8D%EF%BC%8C%E6%88%96%E8%80%85%E6%89%A7%E8%A1%8C%E7%94%A8%E6%88%B7%E8%AF%B7%E6%B1%82%E7%9A%84%E6%93%8D%E4%BD%9C%E3%80%82)。

## 主要组成结构与交互

![https://www.philschmid.de/mcp-introduction](blob:https://chatgpt.com/4c8b4182-7b1e-48a8-a118-42ecfab4bac4) _图1：MCP客户端与服务器的功能概览。MCP服务器负责暴露**工具**、**资源**和**提示**（Prompts），客户端负责调用工具、查询资源并使用提示[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=The%20current%20components%20of%20MCP,servers%20include)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=MCP%20Clients%20are%20part%20of,with%20a%20specific%20MCP%20Server)。底部列出三类功能的示例：工具如检索/搜索、发送消息，资源如文件、数据库记录，提示如文档问答模板等。_

MCP遵循典型的**客户端-服务器**架构，其中包括以下角色[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,AI%20model%20via%20the%20client)：

* **主机（Host）**：运行LLM应用的宿主环境，它是用户直接交互的应用程序（例如 Claude 桌面版、IDE 插件或自定义代理）。主机负责启动并管理 MCP 客户端，与LLM模型交互。
    
* **客户端（Client）**：嵌入在主机应用中的组件，每个客户端与一个MCP服务器保持1:1的连接。客户端按照MCP协议构建请求、发送请求并接收服务器响应，是主机和服务器沟通的桥梁[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,Maintain%20a%201%3A1%20to%20connection)。
    
* **服务器（Server）**：独立运行的外部程序，封装某个数据源或工具的功能，并通过标准化接口将这些**功能**提供给客户端[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,AI%20model%20via%20the%20client)。服务器由数据源/工具提供方实现，典型如文件系统MCP服务器、数据库MCP服务器、GitHub MCP服务器等[anthropic.com](https://www.anthropic.com/news/model-context-protocol#:~:text=The%20Model%20Context%20Protocol%20is,that%20connect%20to%20these%20servers)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=Servers%20can%20be%20built%20in,clients%20primarily%20via%20two%20methods)。服务器自身维护对实际后端服务的访问（例如调用API密钥、数据库连接），因此主机应用无需直接暴露这些敏感信息或细节。
    

在MCP架构中，“**模型上下文**”指的是通过MCP服务器提供给LLM的外部信息和功能性上下文。具体包括三类核心要素[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=The%20current%20components%20of%20MCP,servers%20include)：

* **工具（Tools）**：由LLM驱动调用的功能接口，相当于模型可调用的函数。例如查询天气、执行计算、调用第三方API等操作[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=1.%20Tools%20%28Model,Selected%20before%20running%20inference)。工具通常涉及执行动作或副作用（类似于REST API中的POST操作）。
    
* **资源（Resources）**：由应用或服务器控制的数据接口，提供只读的内容或数据片段，相当于模型可访问的“文件”或数据记录[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=1.%20Tools%20%28Model,Selected%20before%20running%20inference)。资源不产生副作用，类似于REST中的GET，用于将外部信息载入模型上下文中。
    
* **提示（Prompts）**：预定义的提示模板，由用户或开发者控制，用于指导LLM以特定方式与工具/资源交互[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=The%20current%20components%20of%20MCP,servers%20include)。这些提示可以看作是可复用的对话或指令模式，在实际推理开始前由用户选择或应用设定，以优化交互效果。
    

**主机-客户端-服务器-模型之间的交互过程**大致如下：主机应用在需要扩展能力时，可以同时启动多个不同的MCP客户端，每个客户端连接一个相应的服务器，从而让一个LLM应用并行获得多个数据源/工具的支持[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=1,based%20on)。LLM模型在对话中如果需要外部信息或操作，便通过其所在的主机触发相应客户端去调用服务器的功能。例如，当用户询问某仓库的未解决Issue列表时，模型可指示客户端调用GitHub服务器提供的查询工具；客户端发送请求后，服务器与GitHub API交互获取数据，并将结果返回客户端，最终整合进模型的对话上下文，供模型生成回答[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=4,a%20final%20response%20for%20the)。整个过程中，LLM始终通过统一的MCP接口与外部系统交互，而具体数据访问和操作逻辑由服务器端封装实现。这种解耦设计保证了LLM应用扩展新功能时，只需更换或新增符合MCP协议的服务器，而无需改动模型本身或客户端框架。

## 协议通信机制

MCP建立在**JSON-RPC 2.0**消息格式的基础之上，是一种面向请求-响应模式的通信协议[zh.wikipedia.org](https://zh.wikipedia.org/wiki/%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE#:~:text=%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE%20%28MCP%29%20%E7%9A%84%E6%8A%80%E6%9C%AF%E5%AE%9E%E7%8E%B0%E6%97%A8%E5%9C%A8%E4%B8%BA%E5%A4%A7%E5%9E%8B%E8%AF%AD%E8%A8%80%E6%A8%A1%E5%9E%8B%20%28LLMs%29%20%E5%BA%94%E7%94%A8%E4%B8%8E%E5%A4%96%E9%83%A8%E6%95%B0%E6%8D%AE%E6%BA%90%E5%92%8C%E5%B7%A5%E5%85%B7%E4%B9%8B%E9%97%B4%E7%9A%84%E6%97%A0%E7%BC%9D%E9%9B%86%E6%88%90%E6%8F%90%E4%BE%9B%E6%A0%87%E5%87%86%E5%8C%96%E7%9A%84%E6%96%B9%E6%B3%95%E3%80%82,2)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=All%20transports%20use%20JSON,Model%20Context%20Protocol%20message%20format)。这意味着客户端和服务器之间交换的数据包是标准的JSON结构，包含方法名、参数、ID等字段，从而实现调用和应答的对应。MCP支持四类基本的消息类型[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=1,from%20the%20other%20side)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=4.%20Notifications%20are%20one,that%20don%E2%80%99t%20expect%20a%20response)：

* **请求（Request）**：由一端发起的调用请求，要求对方执行某种操作（例如调用某个工具或读取某个资源），请求消息包含`method`（方法名）和`params`（参数）。
    
* **响应（Result）**：对应于请求的成功结果，由被调用方返回。消息内容通常是执行请求所得的数据结果，结构上可以是任意JSON对象。
    
* **错误（Error）**：对应于请求的失败结果。如果请求在服务器端处理失败（如方法未找到、参数错误等），将返回带有错误代码和消息的错误响应[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=enum%20ErrorCode%20,32603)。
    
* **通知（Notification）**：单向消息，发送方不期待回应。例如客户端可以发送`initialized`这类通知告知服务器某些事件。通知仅含方法和参数，没有响应。
    

通信采用**半双工**的模式：双方均可发送请求给对方，彼此响应各自的请求。这允许实现双向交互（例如服务器在某些情况下也可主动请求客户端执行操作或通知事件）。不过在典型场景下，大部分请求由客户端发起、服务器处理。

### 消息流程与数据流

一个完整的MCP对话通常经历以下阶段[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=1)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=1,based%20on)：

1. **初始化（Initialization）**：当主机应用启动或用户开启一个新的LLM会话时，它会创建所需的 MCP 客户端进程。客户端首先通过发送`initialize`请求与服务器进行握手，声明自身支持的协议版本和功能列表；服务器回应自身的协议版本及能力（能力协商）[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=1)。完成握手后，客户端再发送一个`initialized`通知表示已就绪。此阶段确保双方协议兼容，并为后续通信建立上下文[zh.wikipedia.org](https://zh.wikipedia.org/wiki/%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE#:~:text=match%20at%20L184%20MCP%20%E7%9A%84%E6%A0%B8%E5%BF%83%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E5%9F%BA%E4%BA%8E,2)。
    
2. **能力发现（Discovery）**：初始化完成后，客户端通常会发送一系列请求查询服务器提供的功能清单，包括`listTools`（获取可用工具列表及说明）、`listResources`（获取可用资源列表及描述）、`listPrompts`（获取可用提示模板列表）等[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=1,based%20on)。服务器返回这些请求的结果，列出其支持的工具名称、功能描述和所需参数 schema，资源的标识URI、类型和说明，以及可用提示的标识和内容等。通过发现阶段，主机了解到服务器的**能力目录**。
    
3. **上下文准备（Context Provision）**：一旦掌握了服务器支持的工具/资源/提示，主机应用即可将这些**上下文选项**提供给模型使用。例如，主机可能将服务器提供的工具列表解析为模型函数调用接口（如转成OpenAI函数调用的JSON定义），让LLM能够在对话中调用它们[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=the%20server%20offers,based%20on)；对于资源，主机可能会预先加载或提示用户选择某些资源插入上下文；对于提示模板，主机可以让用户选定某个模板来引导模型对话风格[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=the%20server%20offers,based%20on)。这一阶段确保在正式应答用户之前，LLM具备可用的外部上下文接口。
    
4. **调用阶段（Invocation）**：当对话进行中，LLM根据用户请求和上下文判断需要使用某项外部功能时，就会触发一次具体的调用。例如，如果LLM决定调用某工具（如`search_documents`）以检索信息，它会通过主机通知对应的客户端发起该工具的调用请求[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=4,a%20final%20response%20for%20the)。客户端据此构造MCP请求（方法名对应工具标识，如`callTool`，参数包含工具名及调用参数），发送给服务器。
    
5. **执行阶段（Execution）**：服务器接收到调用请求后，解析请求的工具名称和参数，执行预先定义的实际逻辑。例如调用外部API、查询数据库或执行本地计算等，然后将获得的结果数据准备好[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=directs%20the%20Client%20to%20send,result%20back%20to%20the%20Client)。对于资源请求（如`readResource`），服务器则读取相应资源内容。**上下文绑定**在此体现：服务器将外部系统的原始响应转换为适合LLM理解的格式（通常是文本或JSON数据），这就是将外部信息**绑定**到LLM上下文的过程[zh.wikipedia.org](https://zh.wikipedia.org/wiki/%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE#:~:text=%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE%20,%E6%9C%8D%E5%8A%A1%E5%99%A8%E5%88%99%E8%B4%9F%E8%B4%A3%E4%B8%8E%E7%9B%B8%E5%BA%94%E7%9A%84%E5%A4%96%E9%83%A8%E6%95%B0%E6%8D%AE%E6%BA%90%E6%88%96%E5%B7%A5%E5%85%B7%E8%BF%9B%E8%A1%8C%E4%BA%A4%E4%BA%92%EF%BC%8C%E8%8E%B7%E5%8F%96%E6%95%B0%E6%8D%AE%E5%B9%B6%E6%8C%89%E7%85%A7%20MCP%20%E5%8D%8F%E8%AE%AE%E8%A7%84%E8%8C%83%E8%BF%9B%E8%A1%8C%E6%A0%BC%E5%BC%8F%E5%8C%96%EF%BC%8C%E6%9C%80%E5%90%8E%E5%B0%86%E6%A0%BC%E5%BC%8F%E5%8C%96%E5%90%8E%E7%9A%84%E6%95%B0%E6%8D%AE%E8%BF%94%E5%9B%9E%E7%BB%99%20LLM%E3%80%82LLM%20%E6%8E%A5%E6%94%B6%E5%88%B0%E4%B8%8A%E4%B8%8B%E6%96%87%E4%BF%A1%E6%81%AF%E5%90%8E%EF%BC%8C%E5%8F%AF%E4%BB%A5%E5%B0%86%E5%85%B6%E7%94%A8%E4%BA%8E%E7%94%9F%E6%88%90%E6%9B%B4%E5%87%86%E7%A1%AE%E3%80%81%E6%9B%B4%E7%9B%B8%E5%85%B3%E7%9A%84%E5%9B%9E%E5%A4%8D%EF%BC%8C%E6%88%96%E8%80%85%E6%89%A7%E8%A1%8C%E7%94%A8%E6%88%B7%E8%AF%B7%E6%B1%82%E7%9A%84%E6%93%8D%E4%BD%9C%E3%80%82)。
    
6. **响应阶段（Response）**：服务器将执行所得的结果封装为MCP响应消息发送回客户端[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=repo%20%27X%27,result%20back%20to%20the%20Client)。如果执行出错，则返回错误消息。客户端收到响应后，解析出结果数据或错误信息。
    
7. **结果整合（Completion）**：客户端最后将服务器返回的结果传递给主机应用。主机负责将该结果**注入LLM的上下文**中，例如作为对模型的新输入（在对话模式下通常以特定格式插入到对话内容中）[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=6,a%20final%20response%20for%20the)。LLM因此得以利用最新获取的外部信息继续推理，产生最终回答[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=6,on%20the%20fresh%2C%20external%20information)。这一整合过程对用户通常是无感的：用户只会看到模型给出完善的答案，其中已经蕴含了外部数据的价值。
    

在通信过程中，连接的生命周期除了正常的请求应答外，还涵盖**终止**和**错误处理**等机制。例如，任意一端可以随时通过发送特定消息或直接断开传输层连接来终止会话；协议也定义了标准的错误码（例如 JSON-RPC 标准错误码 -32601 方法未找到等）以及扩展错误码范围，用于传递丰富的错误信息[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=enum%20ErrorCode%20,32603)。这些细节保证了协议在异常情况下的健壮性。

### 传输层与通信方式

MCP 将具体的数据传输通道抽象为**传输层（Transport Layer）**，以适应不同应用场景[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=Transport%20layer)。当前协议规范支持两种主要传输机制[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,connection%20using%20the%20SSE%20standard)：

* **标准输入/输出（Stdio）**：客户端和服务器通过进程间标准IO流通信。通常由主机直接启动服务器进程，两者通过 stdin/stdout 管道交换 JSON-RPC 消息。这种方式适合本地部署，延迟低且实现简单，非常适用于在本地访问文件系统、运行脚本等场景[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,events%29%20to%20the)。
    
* **HTTP 长连接（Server-Sent Events, SSE）**：客户端通过HTTP协议连接远程服务器，初始化后采用Server-Sent Events保持长连接，由服务器推送消息给客户端，客户端通过HTTP POST发送请求[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=2)。这种方式适用于远程或分布式场景，利用HTTP的基础设施，并方便结合HTTPS确保传输安全[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=1)。需要注意在该模式下的认证授权、安全控制（例如使用TLS加密、API鉴权等）[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=1)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=,Message%20validation)。
    

传输层的存在使得MCP的应用非常灵活：对于同一套协议逻辑，客户端和服务器既可以在本地通过管道通信，也可以跨网络通过HTTP通信。而对更高层的协议使用者而言，这些传输细节是透明的——开发者只需专注于实现客户端/服务器逻辑，SDK 已经封装了传输的处理。

## 核心架构设计

MCP的架构具有**分层解耦、接口抽象和可扩展**的设计特点[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=The%20Model%20Context%20Protocol%20,core%20architectural%20components%20and%20concepts)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=Transport%20layer)。核心可分为**协议层**和**传输层**：

* **协议层（Protocol Layer）**：负责MCP消息的包装、解析、路由和处理，定义了高层通信模式。它包含如`Protocol`、`Client`、`Server`等抽象类/模块[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=Key%20classes%20include%3A)。协议层基于JSON-RPC规范实现请求/响应的匹配，维护会话状态，支持方法注册与调用分发等。例如，在SDK中`Protocol`类提供`setRequestHandler`注册请求处理器、`request()`发送请求等待结果等接口[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=class%20Protocol,Result%3E%29%3A%20void)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=%2F%2F%20Send%20requests%20and%20await,T)。通过这些抽象，开发者无需手动处理底层JSON消息的封装拆解，只需针对特定方法定义处理逻辑即可。协议层也规定了标准的方法名称（如`initialize`、`listTools`、`callTool`等)和数据结构（使用JSON Schema定义参数和返回格式），确保不同实现之间的互操作性[github.com](https://github.com/modelcontextprotocol/modelcontextprotocol#:~:text=Model%20Context%20Protocol%20specification)[github.com](https://github.com/modelcontextprotocol/modelcontextprotocol#:~:text=The%20schema%20is%20defined%20in,as%20well%2C%20for%20wider%20compatibility)。
    
* **传输层（Transport Layer）**：负责协议消息在客户端和服务器之间的实际发送与接收。正如前述，MCP目前提供了标准IO和HTTP(SSE)两种传输实现，未来也可扩展出新的传输方式（例如 WebSocket 等）。传输层与协议层通过统一接口衔接，使得协议层逻辑独立于具体传输机制[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=The%20transport%20layer%20handles%20the,MCP%20supports%20multiple%20transport%20mechanisms)。这种分层设计借鉴了语言服务器协议（LSP）的架构思想，将通用的协议逻辑与多样的传输通道解耦[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,filesystem%2C%20git%2C%20Slack%2C%20etc)。
    

**抽象接口与可扩展机制**方面，MCP严格定义了客户端和服务器应支持的基础请求/通知集合，同时允许根据需要扩展。例如协议预留了自定义错误码区间，供特定服务器扩展错误类型[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=MCP%20defines%20these%20standard%20error,codes)；又如服务器可以在规范之上增加自定义的资源类型或工具，只要遵循消息格式，客户端依然可以通过能力发现机制识别使用。这种设计保证了协议的演进与向后兼容。值得一提的是，Anthropic 提供的官方规范以TypeScript编写并转换为JSON Schema发布，为各语言实现提供了准确蓝本[github.com](https://github.com/modelcontextprotocol/modelcontextprotocol#:~:text=Model%20Context%20Protocol%20specification)。正因具有完整清晰的规范文档[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=%2A%20Prompts%20%28User,launch%20as%20just%20a%20spec)，MCP在推出后迅速吸引了社区广泛参与实现：目前已官方支持Python、TypeScript、Java、C#、Kotlin、Rust、Swift等语言的SDK[github.com](https://github.com/modelcontextprotocol#:~:text=%2A%20typescript)[github.com](https://github.com/modelcontextprotocol#:~:text=%2A%20python)，并有大量社区维护的服务器插件和工具涌现。这种开放标准+多语言SDK+丰富示例的策略，使MCP在短时间内获得了类似网络效应的普及[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=RPC%202.0%20www.jsonrpc.org%20%20,libraries%2C%20now%20Java%2C%20Kotlin%20C)。

## MCP 与 Python 的集成使用

作为开放生态的一部分，MCP提供了**官方的 Python SDK**（`mcp`库）以便开发者在 Python 环境中轻松创建 MCP 客户端或服务器[github.com](https://github.com/modelcontextprotocol#:~:text=%2A%20python)。该 SDK 遵循Python习惯的接口风格，利用装饰器等特性大幅简化了 MCP 服务器的实现。下面我们以构建一个简单的 MCP 服务器为例，说明其与Python的集成方式：

```python
# server.py - 构建一个简单的 MCP 服务器
from mcp.server.fastmcp import FastMCP

# 创建MCP服务器实例，命名为 "Demo"
mcp = FastMCP("Demo")

# 定义一个加法工具
@mcp.tool()
def add(a: int, b: int) -> int:
    """Add two numbers"""
    return a + b

# 定义一个动态问候语资源
@mcp.resource("greeting://{name}")
def get_greeting(name: str) -> str:
    """Get a personalized greeting"""
    return f"Hello, {name}!"
```

如上所示，我们通过`FastMCP`类创建了名为“Demo”的服务器，然后使用`@mcp.tool()`装饰器注册了一个名为`add`的工具函数，它接受两个整数参数并返回它们的和[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=,FastMCP)。借助装饰器，SDK会自动将该Python函数转换为 MCP 工具的描述（包括名称、参数类型和文档字符串等），供客户端发现和调用[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=,return%20a%20%2B%20b)。类似地，`@mcp.resource()`装饰器将`get_greeting`函数注册为一个资源端点，这里使用了 URI 模板`greeting://{name}`，表示客户端可以通过提供不同的`name`参数来读取不同的问候语内容[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=,Hello%2C%20%7Bname)。这个资源函数简单返回一个个性化问候字符串。通过这种方式，开发者无需手动构造繁杂的协议消息，只需专注于业务逻辑，本例中不到10行代码便完成了一个工具和一个资源的对外暴露。

在实现功能后，**运行与集成**也很方便：开发者可以使用MCP提供的命令行工具将服务器注册到主机应用中。例如，对于Claude桌面版，可直接运行`mcp install server.py`将此服务器安装到Claude中使用[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=You%20can%20install%20this%20server,it%20right%20away%20by%20running)；或者在开发调试时运行`mcp dev server.py`启动服务器并使用“MCP Inspector”可视化工具进行测试[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=Alternatively%2C%20you%20can%20test%20it,with%20the%20MCP%20Inspector)。这些开发工具进一步降低了集成门槛。

除了构建服务器，Python SDK 也支持编写 MCP 客户端，这对于打造自定义的AI应用代理非常有用。例如，可以在Python中启动一个LLM（如通过OpenAI API或本地模型）、再通过`mcp.ClientSession`连接若干现有的MCP服务器，从而让该LLM具备查询数据库、调用Web服务等能力[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=tools%20%3D%20await%20session)。SDK 提供了方便的异步接口来完成这一切：开发者只需提供服务器启动参数（如可执行文件路径等），然后使用`session.list_tools()`、`session.call_tool()`、`session.read_resource()`等方法即可请求服务器功能并获取结果[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,list_prompts)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,file%3A%2F%2Fsome%2Fpath)。例如，调用`session.call_tool("add", {"a":1,"b":2})`会向服务器发出加法请求并返回结果。这种客户端用法意味着开发者可以在纯Python的环境下，将多个现有MCP服务器的能力集成到一个新的LLM应用中，充分发挥各类工具的协同作用。

综上，MCP在Python中的集成表现出**清晰的API设计和良好的开发者体验**：通过多语言SDK和装饰器等高层抽象，开发者可以迅速地将Python函数封装为标准的LLM可用工具/资源[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=Let%27s%20create%20a%20simple%20MCP,calculator%20tool%20and%20some%20data)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,Hello%2C%20%7Bname)。同时，由于协议层的封装，Python客户端也能方便地调用外部能力，将结果拿来与本地LLM结合。从快速开发原型到未来大规模部署，MCP的Python支持为构建“AI+数据/工具”的应用提供了强有力的支撑。

**参考文献：**

1. Anthropic 官方博客: _Introducing the Model Context Protocol_[anthropic.com](https://www.anthropic.com/news/model-context-protocol#:~:text=Today%2C%20we%27re%20open,produce%20better%2C%20more%20relevant%20responses)[anthropic.com](https://www.anthropic.com/news/model-context-protocol#:~:text=MCP%20addresses%20this%20challenge,to%20the%20data%20they%20need)
    
2. Philschmid 博客: _Model Context Protocol (MCP) 概览_[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=Think%20of%20it%20like%20USB,effort%20across%20teams%2C%20inconsistent%20implementations)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=1,based%20on)
    
3. MCP 官方文档: _Core architecture_[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=All%20transports%20use%20JSON,Model%20Context%20Protocol%20message%20format)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=1)；_通信机制_[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=1,from%20the%20other%20side)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=enum%20ErrorCode%20,32603)
    
4. 模型上下文协议维基百科[zh.wikipedia.org](https://zh.wikipedia.org/wiki/%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE#:~:text=%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE%EF%BC%88%E8%8B%B1%E8%AF%AD%EF%BC%9AModel%20Context%20Protocol%EF%BC%8C%E7%BC%A9%E5%86%99%EF%BC%9AMCP%EF%BC%89%E6%98%AF%20Anthropic%20,2%20%5D%E3%80%82)[zh.wikipedia.org](https://zh.wikipedia.org/wiki/%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE#:~:text=%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE%20,%E6%9C%8D%E5%8A%A1%E5%99%A8%E5%88%99%E8%B4%9F%E8%B4%A3%E4%B8%8E%E7%9B%B8%E5%BA%94%E7%9A%84%E5%A4%96%E9%83%A8%E6%95%B0%E6%8D%AE%E6%BA%90%E6%88%96%E5%B7%A5%E5%85%B7%E8%BF%9B%E8%A1%8C%E4%BA%A4%E4%BA%92%EF%BC%8C%E8%8E%B7%E5%8F%96%E6%95%B0%E6%8D%AE%E5%B9%B6%E6%8C%89%E7%85%A7%20MCP%20%E5%8D%8F%E8%AE%AE%E8%A7%84%E8%8C%83%E8%BF%9B%E8%A1%8C%E6%A0%BC%E5%BC%8F%E5%8C%96%EF%BC%8C%E6%9C%80%E5%90%8E%E5%B0%86%E6%A0%BC%E5%BC%8F%E5%8C%96%E5%90%8E%E7%9A%84%E6%95%B0%E6%8D%AE%E8%BF%94%E5%9B%9E%E7%BB%99%20LLM%E3%80%82LLM%20%E6%8E%A5%E6%94%B6%E5%88%B0%E4%B8%8A%E4%B8%8B%E6%96%87%E4%BF%A1%E6%81%AF%E5%90%8E%EF%BC%8C%E5%8F%AF%E4%BB%A5%E5%B0%86%E5%85%B6%E7%94%A8%E4%BA%8E%E7%94%9F%E6%88%90%E6%9B%B4%E5%87%86%E7%A1%AE%E3%80%81%E6%9B%B4%E7%9B%B8%E5%85%B3%E7%9A%84%E5%9B%9E%E5%A4%8D%EF%BC%8C%E6%88%96%E8%80%85%E6%89%A7%E8%A1%8C%E7%94%A8%E6%88%B7%E8%AF%B7%E6%B1%82%E7%9A%84%E6%93%8D%E4%BD%9C%E3%80%82)
    
5. MCP Python SDK 快速开始示例[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=Let%27s%20create%20a%20simple%20MCP,calculator%20tool%20and%20some%20data)[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=,Hello%2C%20%7Bname)
    
6. 雨梦山人: _MCP协议详解：一文读懂模型上下文协议_[cnblogs.com](https://www.cnblogs.com/shanren/p/18797313#:~:text=MCP%EF%BC%88Model%20Context%20Protocol%EF%BC%8C%E6%A8%A1%E5%9E%8B%E4%B8%8A%E4%B8%8B%E6%96%87%E5%8D%8F%E8%AE%AE%EF%BC%89%E6%98%AF%E7%94%B1%20Anthropic%20%E6%8E%A8%E5%87%BA%E7%9A%84%E5%BC%80%E6%BA%90%E5%8D%8F%E8%AE%AE%EF%BC%8C%E6%97%A8%E5%9C%A8%E5%AE%9E%E7%8E%B0%E5%A4%A7%E5%9E%8B%E8%AF%AD%E8%A8%80%E6%A8%A1%E5%9E%8B%EF%BC%88LLM%EF%BC%89%E4%B8%8E%E5%A4%96%E9%83%A8%E6%95%B0%E6%8D%AE%E6%BA%90%E5%92%8C%E5%B7%A5%E5%85%B7%E7%9A%84%E6%97%A0%E7%BC%9D%E9%9B%86%E6%88%90%EF%BC%8C%E7%94%A8%E6%9D%A5%E5%9C%A8%E5%A4%A7%E6%A8%A1%E5%9E%8B%E5%92%8C%E6%95%B0%E6%8D%AE%E6%BA%90%E4%B9%8B%E9%97%B4%E5%BB%BA%E7%AB%8B%E5%AE%89%E5%85%A8%E5%8F%8C%E5%90%91%E7%9A%84%E9%93%BE%E6%8E%A5%E3%80%82)[cnblogs.com](https://www.cnblogs.com/shanren/p/18797313#:~:text=MCP%20%E9%81%B5%E5%BE%AA%E5%AE%A2%E6%88%B7%E7%AB%AF)

* * *

**You:**


重点研究一下提示词模块，提示词的管理等。当然还有提示提与其他模块的关系：tools, resouces等。

* * *

**ChatGPT:**

好的，关于提示词模块（Prompts），为了深入研究它的机制与管理方式，以及它与 tools、resources 的关联，请确认：

1. 是否只需要关注 MCP 中对提示词（Prompt）模块的设计和作用，不涉及 Prompt Engineering 更广义的范畴？
    
2. 是否优先关注 Python SDK 和 Claude 等支持 Prompt 模块的平台的实现方式？
    

确认后我会立刻开始研究。

* * *

**You:**

目前只关注mcp 提示词模块，优先关注python 平台

* * *

**ChatGPT:**

明白了，我将重点研究 MCP 协议中的提示词（Prompt）模块，包括其结构、管理方式、用途，以及它与 tools 和 resources 模块之间的关系。研究将优先聚焦在 Python 平台上的支持与实现方式，包括如何在 Python MCP SDK 中定义、注册和调用提示词。

研究完成后，我会为你整理一份结构清晰的说明文档。

* * *

**ChatGPT:**

# MCP Prompt 模块设计与使用详解

## Prompt 模块的核心作用

**Model Context Protocol (MCP)** 将 **Prompt** 与 Resource、Tool 并列为核心组件，用于定义可重用的**提示模板**和工作流[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20enable%20servers%20to%20define,and%20share%20common%20LLM%20interactions)。Prompt 模块让服务器预先定义一些提示（Prompts），供客户端和LLM重复使用，从而**标准化常见的人机交互**[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20enable%20servers%20to%20define,and%20share%20common%20LLM%20interactions)。这些 Prompt **由用户控制选择触发**，服务器将其暴露给客户端，用户可显式选择使用某个提示模板[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20are%20designed%20to%20be,explicitly%20select%20them%20for%20use)。一个 Prompt 通常包含预设的对话消息或指令，可以视作**封装良好的提示语**，类似于为模型提供的固定“场景”或“API”调用[nshipster.com](https://nshipster.com/model-context-protocol/#:~:text=,Think%3A%20databases%2C%20file%20systems%2C%20documents)[nshipster.com](https://nshipster.com/model-context-protocol/#:~:text=,between%20you%20and%20the%20model)。

**Prompt 的作用特点**包括[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20in%20MCP%20are%20predefined,templates%20that%20can)：

* **接受动态参数**：Prompt 模板可定义参数占位符，调用时由用户提供具体参数值（例如编程语言、文件路径等)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20in%20MCP%20are%20predefined,templates%20that%20can)。
    
* **嵌入资源上下文**：Prompt 可以集成 Resource 提供的内容作为提示的一部分，如嵌入文件内容或日志片段[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,like%20slash%20commands)。
    
* **串联多轮交互**：一个 Prompt 可预定义多条对话消息，实现**多步对话工作流**（如先向用户提问，再根据回答继续）[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,like%20slash%20commands)。
    
* **指导特定任务流程**：通过精心设计提示语，Prompt 可以引导 LLM 按特定步骤完成任务，确保输出符合预期。
    
* **呈现为UI元素**：Prompt 往往会在客户端界面中作为可点击的快捷项出现（如**斜杠指令**、菜单选项等）[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,like%20slash%20commands)。
    

通过 Prompt 模块，开发者可以提前设计好与LLM交互的模板，让**常见需求一键触发**，得到比即兴询问更精确有用的回复[nshipster.com](https://nshipster.com/model-context-protocol/#:~:text=,between%20you%20and%20the%20model)。

## Prompt 与 Tool、Resource 的协作关系及调用流程

**Prompt、Tool 和 Resource**分别代表MCP中三种不同控制方式的能力：Prompt 为**用户控制**，Tool 为**模型控制**，Resource 为**应用控制**[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=API%20interaction%2C%20MCP%20was%20designed,inventing%20everything%20from)。三者相互配合，使LLM既能获取外部数据，又能执行动作，还能通过预设模板优化对话流程：

* **Prompt 与 Tool 的关系**：Prompt 模板可以帮助LLM**更好地调用工具**。实际上，Prompt 常被视为一种由用户触发的特殊“工具”[medium.com](https://medium.com/@nimritakoul01/the-model-context-protocol-mcp-a-complete-tutorial-a3abe8a7f4ef#:~:text=In%20the%20context%20of%20tools%2C,user%20through%20the%20client%20application)。例如，Prompt 可以在提示语中指引模型使用某个Tool或提供调用Tool所需的信息，从而**以最优方式利用服务器提供的工具**[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=3.%20Prompts%20%28User,Selected%20before%20running%20inference)。不过Tool的实际调用仍由LLM自行决定并经用户批准，因此Prompt的作用是引导和规范，而非强制调用。
    
* **Prompt 与 Resource 的关系**：Prompt 可以直接**嵌入资源内容**作为上下文提供给LLM[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20in%20MCP%20are%20predefined,templates%20that%20can)。当 Prompt 需要用到文件、数据库等数据时，服务器可在生成 Prompt 时读取相应Resource，将其内容填充进提示消息。例如预定义的“日志分析”Prompt会取指定时间范围的日志内容和代码文件内容作为消息，提供给模型进行分析[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Embedded%20resource%20context)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D%20%7D)。因此，通过 Prompt，用户无需手动提供长文本，由服务器自动将相关资源注入对话，提高了上下文提供的便利性。
    

**调用流程**方面，Prompt 模块与 Tool、Resource 各有不同的触发时机，但整体遵循 MCP 的请求/响应机制：

1. **发现阶段**：客户端建立连接后，会查询服务器支持的 Prompt 列表。客户端发送`prompts/list`请求，服务器返回可用 Prompt 的清单，包括每个 Prompt 的名称、描述和参数列表[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Clients%20can%20discover%20available%20prompts,endpoint)。例如返回包含 `"name": "analyze-code"`、`"description": "...改善代码"` 以及参数定义等[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=prompts%3A%20%5B%20%7B%20name%3A%20%22analyze,required%3A%20true)。类似地，工具和资源也有各自的`tools/list`、`resources/list`（或通过约定URI形式）供发现[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/tools#:~:text=,calculations%20to%20complex%20API%20interactions)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,list_resources)。
    
2. **用户选择触发**：在客户端应用中，这些 Prompt 通常呈现为按钮、菜单或斜杠命令等UI元素。用户根据需要选择某个 Prompt，并提供必要的参数输入（如果有）。由于 Prompt 是用户主动触发，确保了**用户对上下文注入的控制**[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=API%20interaction%2C%20MCP%20was%20designed,inventing%20everything%20from)。
    
3. **获取 Prompt 内容**：客户端收到用户指令后，调用`prompts/get`请求来获取该 Prompt 的具体内容[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=To%20use%20a%20prompt%2C%20clients,request)。请求中指定 Prompt 名称和参数值，如：
    
    ```json
    {
      "method": "prompts/get",
      "params": {
        "name": "analyze-code",
        "arguments": { "language": "python" }
      }
    }
    ```
    
    服务器处理此请求，在后台按照 Prompt 模板生成对应的提示消息序列。返回结果包含该 Prompt 的描述（可选）和具体对话消息列表[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=%2F%2F%20Response%20%7B%20description%3A%20,n%20%20%20%20for)。例如，上例返回中包含一条用户角色消息，其文本内容为“请分析以下Python代码以改进……”并附上代码片段[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=description%3A%20,nresult)。对于复杂 Prompt，返回的`messages`数组可能包含多条交替的用户/助手消息，用以预先搭建对话流程[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40mcp.prompt%28%29%20def%20debug_error%28error%3A%20str%29%20,)。
    
4. **注入对话并调用模型**：客户端拿到 Prompt 返回的消息后，会将其插入到与LLM的对话上下文中（通常作为用户消息或整个对话前置）。接下来由**LLM根据这些提示消息生成回答**。在回答过程中，如果需要，LLM **可能决定调用工具 (Tool)**。例如，如果 Prompt 引导模型执行某项需要外部操作的任务，模型就会产生调用相应Tool的意图。客户端检测到模型的Tool调用意图，会通过`tools/call`向服务器请求执行该工具[nshipster.com](https://nshipster.com/model-context-protocol/#:~:text=The%20client%20can%20share%20this,request)[nshipster.com](https://nshipster.com/model-context-protocol/#:~:text=prompt%20or%20a%20user%20message,request)。服务器执行Tool逻辑后返回结果，客户端再将结果反馈给模型。整个过程中，Resource 则可能被动提供数据：要么已经由 Prompt 预嵌入，要么模型后续要求读取某个Resource（这需要模型提出请求或用户再次提供）。总之，**Prompt预先铺好了问题背景，Tool提供行动能力，Resource提供事实数据**，三者结合使LLM应用完成复杂任务的过程更加流畅。
    
5. **响应用户**：最终，LLM生成的答案通过客户端返回给用户。一次完整的交互可能就是**用户->Prompt->模型(可能用Tool)->答复**的链条。由于有 Prompt 的介入，用户得到的回复更符合预期格式或经过定制，提高了效率和效果[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=3.%20Prompts%20%28User,Selected%20before%20running%20inference)。
    

值得注意的是，MCP 将 Prompt 定位为**用户发起的交互模板**，必须由用户选择触发，而不像工具那样由模型自主决定[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=API%20interaction%2C%20MCP%20was%20designed,inventing%20everything%20from)。这种设计确保了当外部数据或特定模板介入对话时，用户始终知情并掌控。此外，Prompt 与 Resource、Tool 虽分属不同机制，但在协议层面“本质上都是往上下文窗口填充 token 的手段”，只是用途和调用方式不同[nshipster.com](https://nshipster.com/model-context-protocol/#:~:text=,Think%3A%20databases%2C%20file%20systems%2C%20documents)[nshipster.com](https://nshipster.com/model-context-protocol/#:~:text=The%20distinction%20between%20these%20is,implement%20everything%20as%20a%20tool)。

## Python SDK 中 Prompt 模板的定义、注册与调用

MCP 官方提供了 **Python SDK** 来方便地创建 MCP 服务器，包括定义 Prompt 模板的简洁接口。在Python平台上，开发者可以通过**装饰器或注册函数**两种方式来定义和暴露 Prompt：

* **使用装饰器简化定义**：Python SDK的`FastMCP`框架提供了`@mcp.prompt()`装饰器用于注册 Prompt 模板函数[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40mcp.prompt%28%29%20def%20review_code%28code%3A%20str%29%20,Please%20review%20this%20code%3A%5Cn%5Cn%7Bcode)。开发者只需用该装饰器标记一个函数，并编写生成提示内容的逻辑，即可将其作为 Prompt 暴露。默认情况下，函数名会用作 Prompt 的名称，函数的文档字符串则自动提取为 Prompt 的描述（也可以在装饰器参数中显式指定名称或描述）[zhuanlan.zhihu.com](https://zhuanlan.zhihu.com/p/19707405738#:~:text=%E4%BD%BF%E7%94%A8%E8%A3%85%E9%A5%B0%E5%99%A8%20mcp)。例如，定义两个 Prompt 模板：
    
    ```python
    from mcp.server.fastmcp import FastMCP, Context, Image
    mcp = FastMCP("My App")  # 初始化 MCP 服务器
    
    @mcp.prompt()
    def review_code(code: str) -> str:
        """Review a piece of code and provide feedback."""
        return f"请帮我审查以下代码：\n\n{code}"
    
    @mcp.prompt()
    def debug_error(error: str) -> list:
        return [
            # 多轮交互: 先提供错误信息，然后让助手提问下一步
            { "role": "user", "content": {"type": "text", "text": f"我遇到了这个错误：{error}"}},
            { "role": "assistant", "content": {"type": "text", "text": "我来帮你调试。你已经尝试了哪些方法？"}}
        ]
    ```
    
    上述代码中，`review_code` Prompt简单地将传入的代码字符串嵌入到固定模板中返回（仅一条用户提示消息）[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40mcp.prompt%28%29%20def%20review_code%28code%3A%20str%29%20,Please%20review%20this%20code%3A%5Cn%5Cn%7Bcode)；而`debug_error` Prompt 返回一个消息列表，包含用户和助手的两条消息，实现了一个两步的对话模板[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40mcp.prompt%28%29%20def%20debug_error%28error%3A%20str%29%20,)。使用装饰器时，SDK会自动根据函数签名生成 Prompt 的参数定义（如参数名称和是否必填），并注册到服务器的 Prompt 列表中[zhuanlan.zhihu.com](https://zhuanlan.zhihu.com/p/19707405738#:~:text=%E4%BD%BF%E7%94%A8%E8%A3%85%E9%A5%B0%E5%99%A8%20mcp)。开发者无需手动编写 JSON Schema，SDK已透过类型注解推导参数类型及必需性，并通过函数文档作为描述来完善 Prompt 元数据[zhuanlan.zhihu.com](https://zhuanlan.zhihu.com/p/19707405738#:~:text=%E4%BD%BF%E7%94%A8%E8%A3%85%E9%A5%B0%E5%99%A8%20mcp)。
    
* **底层注册机制**：装饰器背后实际是为 MCP **注册了对应的请求处理器**。MCP 协议规定了两个与 Prompt 相关的请求：`prompts/list` 列出所有 Prompt 模板，`prompts/get` 获取特定 Prompt 内容。Python SDK 提供了等价的装饰器方法，如`@server.list_prompts()`和`@server.get_prompt()`，用于开发者自行实现这两个请求的处理逻辑[oschina.net](https://www.oschina.net/p/modelcontextprotocol-python-sdk#:~:text=%E8%A3%85%E9%A5%B0%E5%99%A8%20%E6%8F%8F%E8%BF%B0%20)。例如，不使用自动装饰器时，可以这样手动注册：
    
    ```python
    server = Server("example-prompts-server", version="1.0.0")
    
    PROMPTS = {
        "example-prompt": types.Prompt(
            name="example-prompt",
            description="这是一个示例 Prompt 模板",
            arguments=[ types.PromptArgument(name="arg1", description="参数1", required=True) ]
        )
    }
    
    @server.list_prompts()
    async def handle_list_prompts() -> list[types.Prompt]:
        # 返回 Prompt 列表
        return list(PROMPTS.values())
    
    @server.get_prompt()
    async def handle_get_prompt(name: str, arguments: dict[str, str] | None) -> types.GetPromptResult:
        if name != "example-prompt":
            raise ValueError("Unknown prompt")
        # 根据名称生成 Prompt 内容
        return types.GetPromptResult(
            description="示例 Prompt",
            messages=[ types.PromptMessage(
                role="user",
                content=types.TextContent(type="text", text=f"示例 Prompt 文本，参数值: {arguments.get('arg1', '')}")
            ) ]
        )
    ```
    
    上述 `handle_list_prompts` 函数直接返回预先定义的 Prompt 列表，`handle_get_prompt`根据请求的 Prompt 名称构造返回内容[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40server.list_prompts%28%29%20async%20def%20handle_list_prompts%28%29%20,types.Prompt)[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40server,types.GetPromptResult)。事实上，如果使用了`@mcp.prompt`装饰器，这些底层方法SDK已自动实现：SDK会在初始化时汇总所有装饰过的 Prompt 函数，内部注册默认的`list_prompts`和`get_prompt`处理器，动态地调用相应的 Prompt 函数并构造消息返回。因此多数情况下开发者**无需手动编写上述样板代码**。可以灵活选择：简单场景用装饰器快捷定义，特殊需求下也可完全自定义 Prompt 的处理过程。
    

一旦服务器端定义并注册了 Prompt 模板，**在客户端即可调用**。Python SDK 提供了对 MCP 客户端的支持，典型用法是使用`ClientSession`对象进行交互[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=async%20with%20stdio_client,initialize)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,list_resources)。例如：

```python
async with ClientSession(read, write) as session:
    await session.initialize()           # 初始化握手
    prompts = await session.list_prompts()      # 列出可用提示:contentReference[oaicite:43]{index=43}
    print(prompts)                       # 查看提示清单

    prompt = await session.get_prompt("example-prompt", arguments={"arg1": "测试"})  
    # 获取指定提示的内容:contentReference[oaicite:44]{index=44}
    print(prompt.messages)               # 输出 prompt 返回的消息列表

    # 后续可将prompt.messages提交给LLM模型进行对话...
```

如上，客户端首先`list_prompts()`获取服务器提示列表，再通过`get_prompt(name, arguments)`取得具体Prompt模板展开后的内容[oschina.net](https://www.oschina.net/p/modelcontextprotocol-python-sdk#:~:text=,list_tools)。在实际应用中，这些调用通常由宿主应用封装，当用户在界面上选择Prompt时触发，无需开发者手动调用。总的来说，**Python SDK 简化了 Prompt 定义和调用流程**：服务器端通过装饰器声明Prompt，客户端则通过统一接口请求，无缝地将Prompt模板融入LLM对话。

## Prompt 模板的组织与管理机制

MCP 为 Prompt 提供了灵活的组织管理策略，使其在不断扩展和演化中保持可用和可控。开发者应关注 Prompt 模板的**命名、作用域、参数化和版本控制**等方面，以便更好地管理 Prompt 库：

* **命名与描述**：每个 Prompt 必须有唯一的名称作为标识符[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Each%20prompt%20is%20defined%20with%3A)。命名应**简洁且具语义**，方便用户理解其用途，例如 `"git-commit"`表示生成Git提交信息[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=if%20%28request.params.name%20%3D%3D%3D%20%22git,request.params.arguments%3F.changes)。建议使用短横线或下划线分隔单词，避免空格。好的命名和详细的描述有助于客户端UI展示和用户选择[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=1,Consider%20prompt%20composability)。开发者应提供清晰的 Prompt 描述，说明其功能和预期输入，使用户一目了然。
    
* **作用域与可见性**：Prompt 的作用域通常是**服务器实例级**的。也就是说，一个 MCP 服务器维护自己的一套 Prompt 列表，仅对连接该服务器的客户端可见。不同服务器的 Prompt 名称互不冲突，但客户端在连接多个MCP服务器时会分别管理各自的 Prompt 列表。需要注意**根（Root）上下文**对 Prompt 的影响：如果客户端指定了操作根（如限定文件系统路径），服务器在生成 Prompt 内容时应尊重这些范围，仅引用根内的资源[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/roots#:~:text=A%20root%20is%20a%20URI,valid%20URI%20including%20HTTP%20URLs)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/roots#:~:text=While%20roots%20are%20informational%20and,not%20strictly%20enforcing%2C%20servers%20should)。尽管协议层未强制Prompt按作用域分类，服务器实现可以根据当前会话的上下文（如用户权限、所选项目等）来**动态调整可用的 Prompt**。例如，只在用户连接特定数据源时才列出相关Prompt。这种粒度控制由服务器逻辑决定。
    
* **参数化**：通过参数机制，Prompt 模板具备**动态生成**能力。定义 Prompt 时可以列出若干参数，每个参数有名称、描述，标明是否必需[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,Whether%20argument%20is%20required)。调用时客户端需提供对应参数值。参数使Prompt更通用可复用，例如同一个“代码审查”Prompt可接受不同代码片段或编程语言作为参数，而不必为每种情况建不同Prompt。MCP在`prompts/list`响应中会公开每个参数名和是否必填，以便客户端提示用户输入[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=prompts%3A%20%5B%20%7B%20name%3A%20%22analyze,required%3A%20true)。Python SDK 会自动根据函数签名生成参数schema，并验证必填参数是否提供[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=1,Handle%20missing%20arguments%20gracefully)。实现Prompt时应做好参数校验和错误处理，当缺少必要参数或格式不对时给出友好错误信息[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=1,Consider%20prompt%20composability)。
    
* **版本控制**：随着应用演进，Prompt 模板可能需要更新优化。**版本管理**可避免改动破坏现有体验。最佳实践是**为 Prompt 模板引入版本号或版本标识**[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=3,Implement%20error%20handling)。一种简单做法是在 Prompt 名称中包含版本号（例如 `"analyze-code-v2"`），或者维护Prompt描述中的版本信息。当对 Prompt 做出非向后兼容的更改时，保留旧版本同时发布新版本，客户端和用户可以平滑迁移。MCP 协议还提供了 Prompt 列表变更的通知机制：服务器支持触发`prompts.listChanged`通知事件[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Servers%20can%20notify%20clients%20about,prompt%20changes)。当 Prompt 集合有新增、移除或修改时，服务器可向客户端发送该通知，提示客户端重新获取 Prompt 列表[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Servers%20can%20notify%20clients%20about,prompt%20changes)。这保证了**Prompt 更新的及时同步**——例如服务器升级了Prompt模板或添加了新Prompt，客户端UI能实时反映变化。
    
* **维护与优化**：管理 Prompt 时还应考虑性能和安全等因素。对于涉及大型资源的 Prompt，可**缓存动态内容**（如日志摘要）以减少每次生成的开销[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=3,Implement%20error%20handling)。同时要实现适当的**权限控制**和**输入清理**：防范用户提供恶意参数导致不安全的资源访问或Prompt注入攻击[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Security%20considerations)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,Document%20security%20requirements)。记录 Prompt 的使用频率和效果，结合用户反馈改进 Prompt 文案也是管理的重要部分。最后，**充分测试**各Prompt在不同输入下的表现，确保其稳健性和正确性[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=1,Consider%20prompt%20composability)。
    

综上，Prompt 模块在 MCP 协议中扮演了**预设对话引导**的角色，通过精心设计与管理 Prompt 模板，开发者可以大幅提升LLM应用与外部工具、数据交互的体验。在 Python SDK 支持下，定义和注册 Prompt 十分方便，同时灵活的组织管理机制保证了 Prompt 库的可扩展性和可靠性。利用好 Prompt 与 Tools、Resources 的协作，能够构建出功能丰富且用户友好的智能应用[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=3.%20Prompts%20%28User,Selected%20before%20running%20inference)。通过标准化的 Model Context Protocol，让不同AI应用都能以一致方式调用这些 Prompt，实现**一次开发、多处复用**，加速AI能力的整合与落地。[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20enable%20servers%20to%20define,and%20share%20common%20LLM%20interactions)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=,AI%20model%20via%20the%20client)

**参考来源：**MCP 官方文档[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20enable%20servers%20to%20define,and%20share%20common%20LLM%20interactions)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompts%20in%20MCP%20are%20predefined,templates%20that%20can)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=To%20use%20a%20prompt%2C%20clients,request)、Python SDK指南[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40mcp.prompt%28%29%20def%20review_code%28code%3A%20str%29%20,Please%20review%20this%20code%3A%5Cn%5Cn%7Bcode)[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40mcp.prompt%28%29%20def%20debug_error%28error%3A%20str%29%20,)、社区技术博客[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=API%20interaction%2C%20MCP%20was%20designed,inventing%20everything%20from)[philschmid.de](https://www.philschmid.de/mcp-introduction#:~:text=3.%20Prompts%20%28User,Selected%20before%20running%20inference)等。上述资料详细阐释了 MCP Prompt 模块的设计思想和实现细节，可供进一步阅读了解。

* * *

**You:**

google 最新的 agent 之间的通信协议深入介绍，架构设计，关键模块，核心思想，还有其他相关的有助于深入理解的关系相关介绍。完整全面，我给出两个网址供参考 1. https://github.com/google/A2A 2. https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/ 第一个应该是一个项目。请进行研究，给我一份报告。可以让我了解这个项目和协议，还有和agent 的作用，我自己也从事相关工作。

* * *

**ChatGPT:**

好的，为了确保这次研究最符合你的需求，我想确认两点：

1. 你希望我重点分析的是 A2A 协议的设计思想、架构、关键模块，以及它如何支持 Agent 之间的通信与互操作性，对吧？是否也希望比较它与其他 Agent 通信机制（如 MCP、LangChain Agent 机制等）的差异？
    
2. 是否需要我额外查阅与 A2A 协议相关的论文、Google I/O 技术演讲或社区解读，以补充 GitHub 和官方博客中未提及的细节？
    

确认后我就可以开始研究撰写全面的技术报告。

* * *

**You:**

重点分析的是 A2A 协议的设计思想、架构、关键模块，以及它如何支持 Agent 之间的通信与互操作性，比较它与其他 Agent 通信机制（如 MCP、LangChain Agent 机制等）的差异，需要额外查阅与 A2A 协议相关的论文、Google I/O 技术演讲或社区解读，以补充 GitHub 和官方博客中未提及的细节

* * *

**ChatGPT:**

明白了，我将深入研究 Google 最新发布的 A2A（Agent-to-Agent）协议，重点分析其设计理念、架构组成、关键模块和通信机制，并详细说明它如何支持 Agent 之间的互操作性。

此外，我还将对比 A2A 与其他主流 Agent 通信协议（如 MCP、LangChain Agent 等）在目标、结构、使用方式等方面的异同。同时，我会查阅相关的论文、Google 官方技术演讲、开发者博客或社区讨论，补充你提供的两个官方链接以外的细节。

整理完成后会提供一份结构化、深入、完整的报告，帮助你快速掌握 A2A 协议的全貌及其与当前 Agent 技术生态的联系。

* * *

**ChatGPT:**

# Google A2A（Agent-to-Agent）协议深度研究报告

## 引言（Introduction）

Google 在 2025 年4月推出了全新的 Agent-to-Agent（简称 A2A）开放协议，旨在让不同平台和厂商构建的 AI **Agent**（智能代理）能够直接通信协作[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=Today%2C%20we%E2%80%99re%20launching%20a%20new%2C,be%20able%20to%20work%20across)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)。在企业环境中，各种自动化 Agent 已广泛用于客服支持、供应链规划、IT 自动化等场景，但一直面临**数据孤岛**和**互操作性**难题——不同框架或厂商的 Agent 彼此无法交流，限制了多智能体系统的潜力[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=To%20maximize%20the%20benefits%20from,term%20costs)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=This%20collaborative%20effort%20signifies%20a,levels%20of%20efficiency%20and%20innovation)。A2A 协议的诞生正是为了提供一个**统一标准**，让异构 Agent 拥有共同语言，在安全可控的前提下协同工作。本文将深入分析 A2A 协议的设计理念、架构组件、通信机制，以及其对异构 Agent 协作的支持，并对比其他相关协议（如 Anthropic 的 Model Context Protocol MCP、LangChain 的 Agent 机制等），最后探讨 A2A 的未来发展方向、生态潜力与局限性。

## A2A协议的设计理念与核心思想

A2A 协议作为**开放标准**，在设计时秉持了五大核心理念，以确保其通用性和实用性[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,%E2%80%9D)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,OpenAPI%E2%80%99s%20authentication%20schemes%20at%20launch)：

* **开放的 Agent 协作能力**：让 Agent 以其自然的非结构化方式协作，即使它们之间不共享内存、工具或上下文[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,%E2%80%9D)。换言之，A2A 不将对方局限为简单的“工具”调用，而是促成真正的多智能体协作。
    
* **构建在现有标准之上**：A2A 基于已有成熟标准实现，如 HTTP、Server-Sent Events (SSE) 以及 JSON-RPC 等[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,stacks%20businesses%20already%20use%20daily)。利用这些广泛使用的技术，便于将 A2A 集成到现有企业 IT 架构中，降低实现成本。
    
* **默认安全（Secure by default）**：协议设计充分考虑企业级鉴权与授权机制，与 OpenAPI 等现有认证方案保持一致等级[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=integrate%20with%20existing%20IT%20stacks,businesses%20already%20use%20daily)。这意味着 Agent 之间通信自带严格的身份验证和权限控制，满足企业安全要求。
    
* **支持长时任务**：A2A 允许 Agent 执行从瞬时响应到耗时数小时、甚至需要人类参与的深度任务[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=%2A%20Support%20for%20long,state%20updates%20to%20its%20users)。在长时任务执行期间，协议提供实时的进度反馈、通知和状态更新能力，以保持各方同步。
    
* **模态无关（Modality agnostic）**：协议并不局限于文本交流，设计之初即支持多种数据模态，包括音频、视频流等[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,including%20audio%20and%20video%20streaming)。因此，Agent 可以通过语言、图像、语音等多媒体形式互动，拓展了协作场景。
    

通过以上理念，A2A 致力于打造一个**通用且安全**的多 Agent 协作框架，使来自不同来源的智能体能够在统一规则下交流信息、分工执行任务，从而释放更高的自治性和生产力[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)。

## 协议架构与主要组成模块

A2A 协议定义了一套明确的**架构组件**和数据结构，保证不同实现之间可以互联互通[github.com](https://github.com/google/A2A#:~:text=,a%20task%20by%20sending%20a)[github.com](https://github.com/google/A2A#:~:text=,a%20task%20by%20sending%20a)。主要模块和概念包括：

* **Agent Card（Agent 卡片）**：每个提供 A2A 服务的 Agent 都公开一个元数据文件（通常托管在其域名下的 `/.well-known/agent.json`）称为 Agent Card[github.com](https://github.com/google/A2A#:~:text=,a%20task%20by%20sending%20a)。该卡片以 JSON 格式描述了该 Agent 的**能力**、提供的技能（功能）、A2A 接口的端点 URL，以及所需的认证方式等[github.com](https://github.com/google/A2A#:~:text=,a%20task%20by%20sending%20a)。其它 Agent（客户端）可以通过读取 Agent Card 来发现此 Agent 的存在及能力，这是**Agent 服务发现**的基础。
    
* **A2A Server**：在 A2A 架构中，负责对外提供服务的 Agent 实例被称为 A2A Server[github.com](https://github.com/google/A2A#:~:text=,a%20task%20by%20sending%20a)。它监听一个标准的 HTTP 接口，按照协议规范实现一系列方法来接受请求、管理任务的执行，并返回结果。简而言之，A2A Server 承担了Agent在协议交互中的服务端角色（如同一个**被调用的Agent**）。
    
* **A2A Client**：发起通信和请求的实体称为 A2A Client，可以是一个应用程序或另一个 Agent[github.com](https://github.com/google/A2A#:~:text=protocol%20methods%20,a%20task%20by%20sending%20a)。Client 根据需要调用 Server 提供的 A2A 接口（例如发送任务请求），相当于**调用方Agent**的角色。需要注意，同一个 Agent 系统既可以对某些任务充当 Server，又可在需要时作为 Client 去调用他人——这取决于具体交互情景。
    
* **Task（任务）**：任务是 A2A 协议中的核心工作单元[github.com](https://github.com/google/A2A#:~:text=,Can%20be)。每当 Client Agent 需要 Remote Agent 执行某项工作时，会通过 A2A 接口发起一个 Task。Task 用唯一的 **Task ID** 标识，并具有生命周期状态，包括 `submitted`（已提交）、`working`（处理中）、`input-required`（等待额外输入）、`completed`（已完成）、`failed`（失败）和 `canceled`（取消）等[github.com](https://github.com/google/A2A#:~:text=,Can%20be)。这样双方可以跟踪任务所处阶段。任务的最终输出称为**Artifact（成果件）**[github.com](https://github.com/google/A2A#:~:text=,g)。
    
* **Message（消息）**：在任务的执行过程中，Client 和 Agent 之间通过交换 Message 来进行对话[github.com](https://github.com/google/A2A#:~:text=,Parts)。每条消息属于某个角色（`user`代表用户/Client侧，`agent`代表Agent侧）并可包含一个或多个 **Part（内容片段）**[github.com](https://github.com/google/A2A#:~:text=,Parts)。消息用于在Agent间传递上下文信息、问题请求、响应答案或其他指令，从而实现**多轮交互**。
    
* **Part（内容部件）**：Part 是消息或 Artifact 中承载具体内容的基本单位[github.com](https://github.com/google/A2A#:~:text=,Parts)。根据类型不同，Part 可分为文本部分（TextPart）、文件部分（FilePart，内嵌二进制数据或提供URI引用），以及数据部分（DataPart，用于结构化的 JSON 数据，如表单）[github.com](https://github.com/google/A2A#:~:text=,Parts)。通过将消息拆分为不同类型的 Part，A2A 能支持丰富的内容格式，并为**用户界面协商**提供基础（详见下文）。
    
* **Artifact（工件/成果件）**：Artifact 指 Agent 在任务过程中生成的输出结果，如文件、图像或结构化数据等[github.com](https://github.com/google/A2A#:~:text=,Parts)。Artifact 内同样由 Part 组成，以描述其内容。Client 在任务完成后可以获取 Artifact 作为最终成果。
    
* **Streaming（流式传输）**：针对可能耗时较长的任务，A2A 支持**流式响应**模式[github.com](https://github.com/google/A2A#:~:text=generated%20files%2C%20final%20structured%20data%29,tasks%2FpushNotification%2Fset)。具体而言，如果 Agent 声明支持“streaming”能力，Client 就可以使用 `tasks/sendSubscribe` 方法订阅任务进度[github.com](https://github.com/google/A2A#:~:text=generated%20files%2C%20final%20structured%20data%29,tasks%2FpushNotification%2Fset)。在任务执行期间，Server 会持续通过 **Server-Sent Events (SSE)** 链接发送 `TaskStatusUpdateEvent`（任务状态更新）或 `TaskArtifactUpdateEvent`（部分成果更新）等事件给 Client[github.com](https://github.com/google/A2A#:~:text=generated%20files%2C%20final%20structured%20data%29,tasks%2FpushNotification%2Fset)。这种机制允许实时获取进展，提高交互的即时性。
    
* **Push Notifications（推送通知）**：除了被动轮询或SSE订阅外，A2A 还支持由 Server 主动向 Client 提供的 webhook URL推送任务更新[github.com](https://github.com/google/A2A#:~:text=%2A%20Streaming%3A%20For%20long,tasks%2FpushNotification%2Fset)。如果 Agent 支持“pushNotifications”能力，Client 可通过调用 `tasks/pushNotification/set` 注册一个回调地址，Server 在任务状态变更时会自动向该地址发送通知。这种模式方便在某些场景下由服务端主动告知结果。
    

上述组件共同构成了 A2A 协议的基础架构。通过 Agent Card 描述能力、使用标准 HTTP(S) 接口和 JSON 数据格式来交换 Task、Message 等对象，A2A 提供了一个清晰的结构，让不同 Agent 实现者遵循共同规范进行开发。这保证了各方**语义一致**且**接口匹配**，为跨平台的 Agent 通信打下基础[github.com](https://github.com/google/A2A#:~:text=One%20of%20the%20biggest%20challenges,%E2%80%93%20all%20while)。例如，一家公司的客服AI Agent（使用某框架）可以通过读取另一家供应链AI Agent（不同框架）的 Agent Card，了解其接口端点和技能，然后按协议发送 Task 请求并收取结果，而无需关心对方内部实现细节。

## Agent间通信与互操作机制的实现方式

![https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/](blob:https://chatgpt.com/2c9ae39c-d72a-4f2b-bffe-ce2ccee3f29f)_图：通过 A2A 协议，客户端 Agent（右侧蓝色机器人）与远程 Agent（左侧绿色机器人）建立安全协作，完成任务管理与状态同步，并协商用户界面呈现格式，实现能力的发现与对接[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,communicate%20with%20the%20remote%20agent)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,replies%2C%20artifacts%2C%20or%20user%20instructions)。图中气泡和图标示意双方交换消息与Artifact，下方四个圆圈对应 A2A 提供的关键功能：安全协作、任务与状态管理、用户体验协商、能力发现。_

A2A 协议规定了**从发现到完成**的一整套交互流程，使两个 Agent 能够顺畅地通信并协同完成任务[github.com](https://github.com/google/A2A#:~:text=Typical%20Flow%3A)[github.com](https://github.com/google/A2A#:~:text=4,canceled)。典型的通信过程如下：

1. **发现（Discovery）**：当一个 Agent 希望调用另一个 Agent 时，首先通过对方公开的**Agent Card**进行发现[github.com](https://github.com/google/A2A#:~:text=Typical%20Flow%3A)。具体来说，Client Agent 知道或被告知了远程 Agent 的标识（例如域名或服务名称），于是请求该地址下的标准路径（如 `https://agent-domain/.well-known/agent.json`）获取 Agent Card。这一步获取的信息包含目标 Agent 支持的能力清单、接口 URL 和认证要求，从而为后续通信做准备[github.com](https://github.com/google/A2A#:~:text=,a%20task%20by%20sending%20a)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,communicate%20with%20the%20remote%20agent)。
    
2. **初始化任务（Initiation）**：Client Agent 准备好任务描述后，向远程 Agent（A2A Server）的接口发送任务请求。主要的方法有两个：`tasks/send`（普通请求）或 `tasks/sendSubscribe`（订阅式请求）[github.com](https://github.com/google/A2A#:~:text=1,object%20in%20the%20response)。请求中包括一个生成的唯一 Task ID、初始的用户消息（Message 对象，通常携带用户的需求或问题）等内容。远程 Agent 收到请求后，将根据任务要求开始处理，并创建对应的 Task 实例。
    
3. **任务处理与消息交换（Processing & Interaction）**：远程 Agent 开始执行任务，可能需要多轮与 Client 的交互。在**非流式**情况下，远程 Agent 可以**同步**完成任务处理并直接在 HTTP 响应中返回最终结果（任务对象，包括状态和输出Artifact）[github.com](https://github.com/google/A2A#:~:text=3.%20Processing%3A%20,object%20in%20the%20response)。而在**流式**模式下（Client 使用了`sendSubscribe`且 Server 支持），远程 Agent 会**异步**处理任务，同时通过先前建立的 SSE 通道不断向 Client 推送事件[github.com](https://github.com/google/A2A#:~:text=3.%20Processing%3A%20,tasks%2FsendSubscribe)。这些事件包含任务当前状态更新或部分产出的Artifact数据，让 Client 实时了解进展。如任务执行需要额外输入（进入`input-required`状态），则暂停等待，此时 Client 可以基于新信息发送后续消息 (`tasks/send`) 补充所需的输入，然后远程 Agent 继续处理[github.com](https://github.com/google/A2A#:~:text=final%20,canceled)。在这个过程中，双方通过**Message**对象来往沟通：Client 发送用户指令或补充信息（角色为"user"的消息），远程 Agent 返回中间回复、提问澄清或最终答案（角色为"agent"的消息）[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,replies%2C%20artifacts%2C%20or%20user%20instructions)。消息中包含的 Part 可以传递文本、文件或数据表单等，使交互内容丰富且结构化。
    
4. **用户体验协商（User Experience Negotiation）**：A2A 允许 Agent 在交流中**协商内容呈现格式**和**UI 能力**[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,video%2C%20web%20forms%2C%20and%20more)。这通过 Message 的 Part 类型和声明的 content type 实现。例如，如果远程 Agent 可以返回一张生成的图像作为结果，它会在消息的 Part 中标记其 MIME 类型为`image/png`，Client Agent 在收到后就能识别这是一个图像，并结合自身或最终用户界面的能力决定如何展示（例如直接显示图片或提供下载链接）。甚至，协议支持双方明确约定用户界面特性，如是否支持内嵌 iframe、视频播放或交互式表单[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,video%2C%20web%20forms%2C%20and%20more)。通过在消息交换中加入这些**UI 元数据**，A2A 实现了对终端用户体验的动态协商，使不同客户端界面的 Agent 也能合作良好，不会因为展示能力差异而影响任务完成。
    
5. **任务完成（Completion）**：经过必要的处理和交互后，远程 Agent 完成任务，将 Task 状态标记为终止（`completed`成功或`failed`失败，或被取消`canceled`）并产出最终 Artifact[github.com](https://github.com/google/A2A#:~:text=4,canceled)。Client Agent 获取到结果后，可将 Artifact 提供给用户或执行后续操作。由于 A2A 规定了标准的任务生命周期状态，Client 可以明确判断任务结束标志，并相应地清理上下文或进行下一步动作。
    

通过上述机制，A2A 实现了**高度灵活**的 Agent 间通信模式。一方面，简单请求-响应可以同步完成，满足快速调用需求；另一方面，对于复杂长时任务，协议又提供了事件流和多轮消息交互的手段，确保 Agent 能持续协作[github.com](https://github.com/google/A2A#:~:text=generated%20files%2C%20final%20structured%20data%29,tasks%2FpushNotification%2Fset)[github.com](https://github.com/google/A2A#:~:text=3.%20Processing%3A%20,tasks%2FsendSubscribe)。值得注意的是，整个交互过程在安全框架下进行——客户端需要满足远程 Agent Agent Card 中要求的认证方式（如 API 密钥、OAuth 令牌等）才能调用其接口[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,stacks%20businesses%20already%20use%20daily)。这样既保证了开放互通，又维护了每个 Agent 服务的访问控制和数据安全。

## A2A协议对异构Agent协同工作的支持

A2A 协议的核心价值在于**打破壁垒，让不同类型、不同厂商的 Agent 协同工作**。其设计从多个方面确保对异构环境的良好支持：

* **框架与厂商无关性**：A2A 完全独立于具体的 AI 框架或供应商，实现了**平台无关的互操作**[github.com](https://github.com/google/A2A#:~:text=One%20of%20the%20biggest%20challenges,%E2%80%93%20all%20while)。协议规范仅依赖通用的 Web 标准（HTTP/JSON），并以开放源代码形式发布[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=We%E2%80%99re%20committed%20to%20building%20the,up%20clear%20pathways%20for%20contribution)。这意味着无论 Agent 背后的实现语言（Python、JavaScript 等）、模型类型（LLM、规则引擎）或托管环境（本地部署、云服务），只要遵循 A2A 接口规范，就能互联。正如官方所指出的，A2A 为企业提供了一种标准方法来管理跨各种平台和云环境的 Agent[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)。企业可以将来自不同供应商的AI能力组合起来，例如将对话式AI客服（厂商A）与财务报表分析Agent（厂商B）通过A2A连接，共同完成一个跨域任务，而不必担心兼容性问题。
    
* **Agent 能力发现与匹配**：通过 Agent Card 描述每个 Agent 的技能和接口，A2A 提供了统一的**能力广告与检索**机制[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=several%20key%20capabilities%3A)。Client Agent 可以在任务开始前获取潜在候选Agent的Agent Card，根据里面列出的**能力**选择最合适者作为合作对象[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,communicate%20with%20the%20remote%20agent)。例如，一个调度Agent需要处理语音命令，就会优先寻找Agent Card中声明了语音识别或处理能力的Agent协助。这种显式的能力描述，方便了异构Agent间的理解和分工，使它们能够**按专长协同**，而不是各自为战。
    
* **多模态与富媒体支持**：A2A 天生支持文本、文件、图像、音视频等多种数据形式[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,including%20audio%20and%20video%20streaming)。这对于异构Agent协作尤为关键，因为不同Agent可能擅长处理不同模态的信息（如视觉AI、语音AI等）。通过Part机制和内容类型协商，一个图像分析Agent可以将结果直接作为图像Part发给另一文本摘要Agent，由后者生成报告[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,video%2C%20web%20forms%2C%20and%20more)。即便它们处理的信息模态不同，也能在A2A框架下顺畅对接。这种模态无关性拓宽了异构Agent共同工作的场景，让组合应用的边界大大扩展。
    
* **长生命周期和异步交互**：A2A 考虑到一些企业流程涉及**长生命周期**的事务（如审批流程、项目管理任务），提供了任务保持和异步通知等功能[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=%2A%20Support%20for%20long,state%20updates%20to%20its%20users)[github.com](https://github.com/google/A2A#:~:text=generated%20files%2C%20final%20structured%20data%29,tasks%2FpushNotification%2Fset)。异构Agent可以在长时间运行的任务中保持通信，不会因为单次请求超时或断开而中断合作。例如，一个采购审批Agent向另一个风险评估Agent发出请求后，可通过 SSE 实时获取评估进展，并在几小时后拿到最终风险报告Artifact。这种持久的协作能力对于复杂业务流程中的Agent联动非常重要，A2A通过协议层支持使之成为可能。
    
* **广泛的生态参与**：Google 推出 A2A 时就联合了超过50家技术合作伙伴共同参与设计和支持[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=Today%2C%20we%E2%80%99re%20launching%20a%20new%2C,be%20able%20to%20work%20across)。这些伙伴涵盖企业软件（Atlassian、Salesforce、SAP、Workday 等）、AI技术公司（Cohere、LangChain 等）、云服务商以及咨询服务商等多个领域[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=Today%2C%20we%E2%80%99re%20launching%20a%20new%2C,their%20entire%20enterprise%20application%20estates)。广泛的行业参与确保了协议在**兼容各种现有系统**时考虑周全，也意味着很快会有多种语言的开源实现、工具包和示例可用[github.com](https://github.com/google/A2A#:~:text=,Genkit)。例如，已有社区开发了 A2A Agent 开发工具包（ADK）以及将 A2A 集成至 LangChain 等框架的样例[github.com](https://github.com/google/A2A#:~:text=%2A%20Multi,A2A%20and%20MCP)。这种生态潜力让异构Agent协作更加易于落地——开发者可以方便地将自家Agent升级支持A2A，与外部Agent互动，从而构建出“AI代理的生态系统”。
    

综上，A2A 的诸多特性共同确保了无论 Agent 在何处、用何种技术栈实现，都能平等对话、协作完成任务。正如 Google 所言，A2A 为 AI Agent 提供了一个“通用语言”，使它们在各自生态中仍能彼此理解并协同工作[github.com](https://github.com/google/A2A#:~:text=One%20of%20the%20biggest%20challenges,%E2%80%93%20all%20while)。这被认为是充分发挥 AI 协作潜能的关键，也是企业用户所期待的跨系统智能自动化的基础[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)。

## 与其他 Agent 通信协议的对比

A2A 并非唯一旨在促进 AI Agent 协作的机制。下面对比 A2A 与另外两个相关的方案——Anthropic 提出的 **MCP（Model Context Protocol）** 以及业界流行的 **LangChain Agent 机制**，从设计目标、调用模式、上下文处理、通信接口等角度分析它们的异同。

### A2A vs Anthropic 模型上下文协议 (MCP)

**Anthropic 的 Model Context Protocol (MCP)** 是另一项面向多智能体/多工具互动的规范。根据 Google 的说明，MCP 主要侧重于为 Agent 提供有用的**工具和上下文**[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)。也就是说，MCP 的关注点在于如何将外部知识、环境或工具接入到单个 Agent（尤其是大型语言模型）的上下文中，使其决策和回答更准确。这与 A2A 的定位形成了**互补**[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)：

* **设计目标区别**：MCP 的目标是标准化**模型上下文接入**，让不同来源的辅助信息（如检索到的文档、数据库查询结果、计算工具）能以统一方式提供给 AI 模型使用。而 A2A 的目标是规范**Agent 与 Agent 之间的通信**，让各自独立运作的 Agent 能直接对话协作[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)。前者聚焦于丰富单个 Agent 的能力，后者则关注多个 Agent 的协同。
    
* **调用模式**：采用 MCP 时，一个 Agent（通常是LLM）会在需要时调用预定义的工具接口或请求上下文信息，这可能通过在对话中以特殊格式插入指令或通过预设的API来实现。而 A2A 则采用**客户端-服务端**模式的直接HTTP通信：一个Agent明确地作为客户端请求另一个Agent执行任务，双方通过 Task/Message 对象交流。在 A2A 中，Agent 间关系更对等，各自作为服务提供特定技能；而 MCP 更像 Agent 对外部插件的**函数调用**模式。
    
* **上下文处理**：MCP 关注如何将**知识/context注入**模型，例如通过设定统一的提示模板、约定工具使用语法等，使模型在生成回答时综合这些额外信息。A2A 则假定每个 Agent 内部有自己维护的上下文和状态，协议不直接干预模型的提示，而是通过消息让 Agents 交流**高层结果**。因此，MCP 的上下文多在**模型内部**（prompt级别）处理，而 A2A 的上下文在**Agent之间**通过结构化消息传递。举例来说，使用 MCP，一个QA代理可能在内部调用网络搜索工具获取资料然后再答复；使用 A2A，则可能是直接把问题发送给一个专门的知识库Agent，由后者返回答案Artifact。
    
* **通信接口**：MCP 尚处于发展阶段，其具体接口可能依赖Anthropic提供的实现，例如通过Claude模型的API来调用工具或注入信息（类似 OpenAI 函数调用的风格）。接口形式可能是隐藏在模型交互中的“隐式协议”。相比之下，A2A 提供了显式的HTTP接口和 JSON 模式，独立于任何特定模型供应商。A2A 通信更像常规的web服务交互，有明确的请求/响应格式和可预测的行为；而 MCP 由于围绕模型，通信可能更多体现在 prompt 设计和模型输出结构上，**标准化程度**和可见性略低。
    
* **互补关系**：值得强调，A2A 与 MCP **并非竞争关系**，而是定位互补[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)。A2A 协议文本指出它是对 MCP 的补充；可以设想在未来的体系中，一个 Agent 可以同时实现 MCP 和 A2A：对内通过 MCP 集成各种工具丰富能力，对外通过 A2A 与其他 Agent 协作完成更复杂的任务。例如，一个智能助理Agent内部用 MCP 接入了日历查询和邮件发送功能，同时它自身又作为A2A Server，被另一项目管理Agent调用来安排会议。这种组合将实现全面的协同AI生态。
    

总的来说，MCP 着眼于**让单个智能体变得更聪明**，而 A2A 着眼于**让多个智能体一起变得更聪明**。两者可以结合，使每个 Agent 即能利用各种工具和上下文，又能与其它 Agent 分工合作。

### A2A vs LangChain 的 Agent 机制

**LangChain** 是一个用于构建以大型语言模型为中心的应用的框架，其中提供了 “Agents” 概念，用来让 LLM 驱动地调用一系列工具来完成任务。在LangChain中，Agent 机制与 A2A 存在明显区别：

* **设计目标**：LangChain Agent 的初衷是**简化 LLM 调用工具的流程**，针对的是**单个智能体内部**如何有逻辑地使用多个工具或知识源。例如，一个 LangChain Agent 可以有浏览器、计算器等工具，LLM 通过决策哪一步用哪个工具来逐步解决问题。而 A2A 的目标是建立**多个独立Agent间**的通信标准。因此，LangChain 的关注点更多在**框架层**（帮助开发者构建复杂链式调用的Agent），而 A2A 是在**分布式系统层**（规范不同系统间通信）。
    
* **调用模式**：LangChain Agent 通常遵循 ReAct 等推理模式，LLM 在内部以**对话形式**逐步决定行动（Action）和观察结果（Observation）[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=LangChain)。比如它生成一段文本“Action: 用浏览器搜索X”，LangChain框架解析后调用浏览器工具，拿到结果再反馈给LLM，循环往复，直到LLM输出“Final Answer”。整个过程是在**本地进程内**进行的，通信发生在LLM和框架代码之间。相较之下，A2A 的调用是**显式网络请求**模式，一个Agent直接通过HTTP请求另一个Agent服务，不涉及在同一LLM对话里嵌入指令，而是**真正的服务对接**。每一次 A2A 调用都是一个明确的Task交易，具有独立的生命周期。
    
* **上下文处理**：在 LangChain 中，Agent 通过内置的 **记忆**（Memory）机制和 prompt 模板在LLM中维护上下文。例如，它可以在每轮对话中把先前工具结果嵌入下一个prompt，让模型记住过程。上下文主要在**LLM prompt级**维护。而在 A2A 中，每个 Agent 自己管理上下文状态（可以是对话历史、数据库等），Agents 之间通过消息传递必要的信息片段。A2A 不要求共享记忆，正如其设计原则所述，Agent 可以在**不共享内存和工具**的情况下自然互动[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,%E2%80%9D)。因此，LangChain Agent 假设有一个**统一的大脑（LLM）**贯穿始终，而 A2A 则是**多大脑各司其职**，通过沟通协调。
    
* **通信接口**：LangChain 本质上是一个Python/JS框架库，没有定义跨网络的协议接口。其 Agent 使用和工具调用都是框架内函数调用或API调用（例如OpenAI API）实现，没有标准化的外部通信协议。要让LangChain Agent与外部系统互动，通常需要自行设计API或借助聊天模型插件接口等。而 A2A 则提供了开箱即用的**标准接口**。值得一提的是，LangChain 已经意识到 Agent 间直接通信的重要性，作为 A2A 的合作伙伴之一，他们表示“多个 Agent 互相交互是未来趋势，非常高兴与 Google Cloud 合作制定共享协议”[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=LangChain)。这意味着未来 LangChain 可能通过集成 A2A，让不同 LangChain Agent 或 LangChain Agent 与其他系统的Agent直接对话，而不仅仅局限在单进程内的工具调用。
    
* **适用场景**：LangChain Agent 机制适用于单个应用中需要多步推理、多工具协作的情形，开发者将其嵌入应用代码，驱动一个LLM完成复杂任务。而 A2A 更适合**跨应用、跨团队甚至跨组织**的Agent协作场景。例如，不同部门各有一个专能Agent，需要彼此调用对方能力时，用A2A更恰当；而在一个后端服务里用LLM orchestrator调用几个内部函数，LangChain Agent足矣。可以认为 LangChain Agents 是**实现单Agent智能**的框架，而 A2A 是**实现多Agent互联**的协议。
    

总之，LangChain 的 Agent 机制与 A2A 所处的抽象层面不同：前者在应用内部的代理决策层，后者在跨系统通信层。两者并不冲突，反而可能结合——未来一个用LangChain构建的Agent完全可以通过A2A开放接口供其他Agent调用，正如LangChain创始人所展望的那样[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=LangChain)。

## A2A的未来发展方向、生态潜力与局限性

作为一项新兴的开放协议，A2A 在发布初期展现出广阔的前景，但也面临一些挑战。以下是对其未来发展的展望、潜在生态影响以及目前的局限性的分析：

* **未来功能改进**：根据官方路线图，A2A 协议本身将持续演进。例如，在**Agent 发现**方面，计划将认证机制和可选凭据直接纳入 Agent Card，使自动发现对接更加方便安全[github.com](https://github.com/google/A2A#:~:text=Protocol%20Enhancements%3A)。在**Agent 协作**方面，正在研究添加 `QuerySkill()` 方法，以便客户端Agent能动态探询远程Agent是否具备某项未在Agent Card明示的技能[github.com](https://github.com/google/A2A#:~:text=,conversation)。此外，还有对任务生命周期与用户体验的改进设想，如支持任务中途动态添加新的交互模态（比如对话过程中切换加入语音/视频）[github.com](https://github.com/google/A2A#:~:text=,reliability%20and%20push%20notification%20mechanisms)。这些改进将进一步增强协议的灵活性和完备性，使其适应更多元的协作需求。
    
* **生态系统潜力**：A2A 身后有 Google 及众多行业领军者支持，发布即以开源形式开放，并鼓励社区参与[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=We%E2%80%99re%20committed%20to%20building%20the,up%20clear%20pathways%20for%20contribution)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=We%27re%20thrilled%20to%20have%20a,the%20future%20of%20AI%20interoperability)。这为其生态繁荣奠定了基础。可以预见，将会出现各种语言的 A2A 客户端/服务端 SDK、测试工具，Agent 市场和注册中心，以及面向特定领域的协议扩展等。例如，大厂如 Salesforce、SAP 等可能将自家平台的AI功能通过A2A开放接口，让客户的其他Agent能够调用；创业公司则可能构建聚合多个第三方Agent的“AI中间件”服务。随著越来越多 Agent 支持 A2A，一个**互联Agent网络**（AgentNet）有望形成，企业可以像调用微服务那样调度各种智能Agent完成端到端业务流程。这种生态一旦形成，将极大降低企业采用多Agent系统的门槛，催生新的解决方案和商业模式（例如专注提供某类Agent能力的供应商，通过优秀表现获得大量其他Agent的调用）。
    
* **标准化与互通效应**：A2A 的出现有望推动业界在 Agent 通信上走向标准化，这和过去Web服务采用REST/API标准所带来的效应类似。有了共同协议，**兼容性问题减少**，Agent 开发者可以专注提升自身技能而非解决对接问题。长期看，这种标准化也可能倒逼其他类似尝试（如OpenAI插件协议、微软Guidance/HuggingGPT等）向互通融合，最终对用户和开发者都有利。当然，A2A 需要通过实际应用检验其通用性是否足够，标准是否需要进一步调整。但只要社区协作推进，其成为事实标准的概率很大。
    
* **当前局限与挑战**：作为一个仍在“草案规范”阶段的协议[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=We%E2%80%99re%20committed%20to%20building%20the,up%20clear%20pathways%20for%20contribution)，A2A 目前也存在一些局限性。首先，**实现成本**是初期障碍——尽管基于HTTP标准，但为现有Agent系统增加A2A支持仍需要开发工作，包括实现协议方法、认证集成、Agent Card编写等。中小型团队可能暂时观望。其次，**性能和效率**需要评估：Agent间通过网络调用在引入灵活性的同时也增加了通信延时和复杂性。如果任务可以在单Agent内用工具完成，是否有必要拆成多Agent？过度网络交互可能带来开销和故障点，这需要在架构设计时权衡。第三，**安全与信任**方面，当众多Agent互联时，如何防范恶意Agent滥用接口、保证数据隐私也是挑战。A2A 提供了鉴权机制，但在跨组织调用时仍需建立信任关系（可能通过签名、可信第三方等方式）。此外，**协议成熟度**尚需提升，目前一些高级功能在计划中（如前述QuerySkill等），可能现阶段无法完全覆盖所有场景，早期采用者可能遇到特殊需求需要自行扩展。最后，**竞争与协调**也是因素——业界如果出现多个类似标准，可能分散注意力。不过，Google 已经联合主要玩家，降低了这一风险。
    
* **推出生产就绪版本的时间表**：Google 宣布正与合作伙伴努力，在**今年稍晚**推出 A2A 的生产可用版本[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=Review%20the%20full%20specification%20draft%2C,learn%20how%20you%20can%20contribute)。这意味着接下来几个季度，我们将看到协议规范冻结1.0版、参考实现完善以及正式部署案例落地。在此过程中，社区反馈将起重要作用，Google 也通过 GitHub Discussion 和 Issues 征集改进意见[github.com](https://github.com/google/A2A#:~:text=Contributing)。短期内，A2A 的演进速度会很快，但也需要关注向后兼容和稳定性，以便早期试用者的系统能够平滑升级。
    

**结语**：A2A 协议代表了AI代理互操作领域的一次重要迈进。通过规范化Agent间通信，它为构建**多Agent协同**的复杂AI系统铺平了道路。尽管仍处于早期阶段，A2A 已展示出成为事实标准的潜力，尤其是在众多领军企业的支持下，其生态有望快速扩张。当然，我们也需理性看到，技术标准的成熟和普及需要时间，过程中需要不断完善和协调各方利益。未来几年，如果 A2A 得以成功推广，我们或许将迎来一个“代理横联”的新时代——各种智能代理像今天的微服务一样，在企业内外**无缝协作**，共同完成单一Agent难以胜任的复杂任务，为生产力和创新带来新的飞跃[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=address%20the%20challenges%20we%20identified,potential%20of%20collaborative%20AI%20agents)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20has%20the%20potential%20to,problems%20and%20enhance%20our%20lives)。

**参考资料：**

1. Google Developers Blog: _“Announcing the Agent2Agent Protocol (A2A)”_, Apr 09, 2025[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=Today%2C%20we%E2%80%99re%20launching%20a%20new%2C,be%20able%20to%20work%20across)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=A2A%20is%20an%20open%20protocol,believe%20this%20universal%20interoperability%20is)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,communicate%20with%20the%20remote%20agent)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,replies%2C%20artifacts%2C%20or%20user%20instructions)等
    
2. Google/A2A GitHub 仓库: _“Agent2Agent Protocol – README and Specification Draft”_[github.com](https://github.com/google/A2A#:~:text=,Tasks%20have%20unique%20IDs%20and)[github.com](https://github.com/google/A2A#:~:text=,Parts)[github.com](https://github.com/google/A2A#:~:text=Typical%20Flow%3A)等
    
3. 合作伙伴引述: Harrison Chase (LangChain) 等对 A2A 的评价[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=LangChain)
    
4. _...以及相关技术社区讨论_（省略）[github.com](https://github.com/google/A2A#:~:text=One%20of%20the%20biggest%20challenges,%E2%80%93%20all%20while)[developers.googleblog.com](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/#:~:text=,%E2%80%9D)等

* * *

**You:**

可以从开发的角度深度调研一下 mcp 吗？开发语言主要是 python ? 整理一份开发文档，指导开发，同时也更容易了解 mcp 相关的开发流程和关键步骤。

* * *

**ChatGPT:**

目前我的研究工具暂时无法使用，但我可以直接为你整理一份**面向 Python 开发者的 Model Context Protocol (MCP) 开发文档**，涵盖从开发环境准备、服务端与客户端开发流程，到调试工具、目录组织和部署建议，帮助你系统掌握 MCP 协议的实际开发方法。

请稍等片刻，我会为你准备一份完整的文档草稿。你是否有偏好的格式？比如：

* 是否希望内容分章节（例如 Markdown 风格）？
    
* 是否打算将该文档纳入项目工程（例如放入 `docs/` 目录）？
    

确认后我将整理内容。如果你暂无特别要求，我将默认使用清晰分节的 Markdown 文档风格来编写这份“Python MCP 开发指南”。

* * *

**You:**

使用清晰分节的 Markdown 文档风格来编写这份“Python MCP 开发指南”，系统环境为 linux或者 macos

* * *

**ChatGPT:**

我已经为你创建并整理好《Python MCP 开发指南》文档，采用清晰的 Markdown 风格，涵盖了从开发准备、服务端与客户端开发、调试工具到目录结构与开发建议等内容，适配 Linux 与 macOS 环境。

如你后续需要添加实战案例、部署脚本、示例代码或扩展为 API 文档，也可以在此基础上继续补充。是否希望我接下来加上某些具体案例（例如 prompt 动态生成、tool 参数校验等）？

* * *

**You:**

1. 前提资源和主要参考例子。
java skd https://github.com/modelcontextprotocol/java-sdk/tree/main/mcp, 相关的测试例子。
python sdk https://github.com/modelcontextprotocol/python-sdk?tab=readme-ov-file。

mcp 官网介绍 https://modelcontextprotocol.io/introduction, 官方网址 https://github.com/modelcontextprotocol。
最新的mcp josn schema https://github.com/modelcontextprotocol/modelcontextprotocol/blob/main/schema/2025-03-26/schema.json。

2. 背景
   现在需要用 c++ 语言开发一个 mcp 的客户端和服务端的 sdk。或者是优先实现prompt 模块的客户端和服务端。
   目标：需要了解 prompt 和那些模块相关，需要什么资源。客户端和服务端需要提供哪些接口。可以优先回答和 prompt 相关的。
   prompt 中涉及资源嵌入等时怎么处理呢?
   现有资源：已经按照最新的 mcp json schema 定义了  c++ 相关的 struct,  实现了定义的 struct 和 json 对象的相互转换。
   目前的资源连接 https://github.com/supertomato696/task/tree/main/mcp_prompt_cpp。
   


请根据相关网址进行调研mcp prompt 模块，尤其是这个模块和哪些模块相关呢? 都是怎么交互的？2. 重要的是 涉及到资源的嵌入时， 有 uri 时， 各种资源，图片，音频，视频等。AudioContent， TextContent，ImageContent，Resource，ResourceContents，ResourceTemplate，TextResourceContents，BlobResourceContents，EmbeddedResource 等作用的场景。好像都是 base64？ 3.服务端通过 Uri 获取相应的 base64 数据，返回给客户端，客端端在调用模型的接口时， base64 数据需要还原吗？模型的多模态应该不是使用 base64 数据。此时整个 mcp 的流程是什么样的呢？mcp 官网提供的 python 和 Java skd 是怎么做的呢？4. 如果是本地文件的 uri ，此时是什么流程呢？如果是不同住机上的 Uri ,此时又是什么流程呢？ 5. 资源嵌入这个的完整流程是什么呢？从客端 端请求到服务端返回，再到客端端调用模型的多模态。

* * *

**ChatGPT:**

为确保我准确回答，请确认以下几点：

1. 是否希望本次研究以 Prompt 模块为主，深入分析它与 Resource、Content 等模块的关系？
    
2. 是否希望我明确从 C++ 实现角度，梳理 Prompt 客户端与服务端交互所需接口、JSON Schema 结构中资源嵌入字段如何处理、跨机/本地 URI 的解析流程？
    
3. 是否需要我对现有 Python 和 Java SDK 的处理方式进行对比分析，用于参考 C++ SDK 设计？
    

确认后我会尽快开展调研并整理成结构清晰的报告文档。

* * *

**You:**

1 本次研究以prompt 模块为主，深入分析它与其他模块的关系 2. 需要从c++的角度梳理Prompt 客户端与服务端交互所需接口、JSON Schema 结构中资源嵌入字段如何处理、跨机/本地 URI 的解析流程 3. 需要我对现有 Python 和 Java SDK 的处理方式进行对比分析，用于参考 C++ SDK 设计

* * *

**ChatGPT:**

明白了，我将围绕 Prompt 模块展开深入研究，重点从 C++ SDK 实现角度梳理其与资源模块的交互关系与数据处理方式。

具体包括：Prompt 与其他模块的结构关联与数据流交互；客户端和服务端在处理 Prompt（尤其包含资源引用时）所需的接口与处理流程；MCP JSON Schema 中如 TextContent、ImageContent、EmbeddedResource 等内容类型的使用场景与 base64 数据的处理；本地 URI 与跨主机资源访问流程；以及 Python 与 Java SDK 中对应功能模块的实现方式与交互对比。

我将结合你提供的示例工程与文档，整理成一份完整的技术分析报告，便于指导后续的 C++ SDK 开发工作。

* * *

**ChatGPT:**

# MCP Prompt 模块与资源内容交互深度解析

## 背景与Prompt模块概述

Model Context Protocol（MCP）是用于标准化应用向大模型提供上下文的开放协议[modelcontextprotocol.io](https://modelcontextprotocol.io/introduction#:~:text=MCP%20is%20an%20open%20protocol,different%20data%20sources%20and%20tools)。在MCP架构中，Server可以提供三类核心能力：**Resources（资源）**、**Tools（工具）**和**Prompts（提示词模板）**[developer.aliyun.com](https://developer.aliyun.com/article/1662194#:~:text=%E6%A0%B9%E6%8D%AEMCP%E5%8D%8F%E8%AE%AE%E5%AE%9A%E4%B9%89%EF%BC%8CServer%E5%8F%AF%E4%BB%A5%E6%8F%90%E4%BE%9B%E4%B8%89%E7%A7%8D%E7%B1%BB%E5%9E%8B%E7%9A%84%E6%A0%87%E5%87%86%E8%83%BD%E5%8A%9B%EF%BC%8CResources%E3%80%81Tools%E3%80%81Prompts%EF%BC%8C%E6%AF%8F%E4%B8%AAServer%E5%8F%AF%E5%90%8C%E6%97%B6%E6%8F%90%E4%BE%9B%E8%80%85%E4%B8%89%E7%A7%8D%E7%B1%BB%E5%9E%8B%E8%83%BD%E5%8A%9B%E6%88%96%E5%85%B6%E4%B8%AD%E4%B8%80%E7%A7%8D%E3%80%82)。其中，**Prompt 模块**用于定义可重用的提示词模板或多步对话流程，便于客户端让用户或LLM选择并使用[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Create%20reusable%20prompt%20templates%20and,workflows)[developer.aliyun.com](https://developer.aliyun.com/article/1662194#:~:text=)。Prompt通常包含预定义的消息序列，可以接受参数填充，并可引用资源内容作为上下文。

Prompt 模块被设计为**用户控制**（user-controlled）的能力，即Server暴露Prompt列表供客户端和用户选择使用；相比之下，Tools模块是**模型控制**（model-controlled）的，用于让LLM主动调用操作[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/tools#:~:text=Tools%20are%20designed%20to%20be,the%20loop%20to%20grant%20approval)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/tools#:~:text=Like%20resources%2C%20tools%20are%20identified,or%20interact%20with%20external%20systems)。Prompt模板可以嵌入资源内容，指导LLM完成特定任务。例如，一个“分析项目”Prompt模板可动态插入日志文件和代码文件内容，让LLM综合分析[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,n%20%20%20%20pass)。下面将详细解析Prompt模块与Resource、Content等模块的结构关系和数据交互机制，并给出C++ SDK实现Prompt模块时的设计要点。

## Prompt模块与Resource/Content模块的结构关系

MCP通过JSON模式和类型定义了Prompt及相关内容的结构。每个Prompt由唯一名称、可选描述和参数列表定义[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Prompt%20structure)。Prompt返回的具体内容由**PromptMessage**列表表示，每条消息包含角色（role）和内容（content）两部分[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=ConfigDict,list%20of%20prompts%20it%20offers)。**PromptMessage.content**可以是三种类型之一：**TextContent**（文本内容）、**ImageContent**（图像内容）或**EmbeddedResource**（嵌入的资源）[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=ConfigDict,to%20a%20prompts%2Fget%20request%20from)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=LLM%20API,extra%3D%E2%80%9Dallow%E2%80%9D%29%20class)。这体现了Prompt模块对Resource模块和Content类型的依赖关系：

* **TextContent**：表示纯文本消息内容，字段包括`type: "text"`和实际文本`text`[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextContent,MIME%20type%20of%20the%20image)。可选的`annotations`用于提供一些渲染或用途提示。
    
* **ImageContent**：表示图像（二进制）消息内容，包含`type: "image"`，`data`（图像数据的base64编码串）以及`mimeType`（图像MIME类型，如`image/png`）[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextContent,%E2%80%9C%E2%80%9D%E2%80%9D%20annotations)。`data`字段承载经Base64编码的图像数据[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=%3D%20None%20model_config%20%3D%20ConfigDict,%E2%80%9C%E2%80%9D%E2%80%9D%20annotations)。
    
* **AudioContent**：类似ImageContent，用于音频数据，包含`type: "audio"`，`data`（base64编码音频数据）和`mimeType`（音频MIME类型）。例如，Rust SDK中AudioContent的数据字段被注释为Base64编码音频数据[docs.rs](https://docs.rs/agentic/latest/agentic/mcp/struct.AudioContent.html#:~:text=%C2%A7%60data%3A%20Vec)（即JSON中传输为字符串）。
    
* **EmbeddedResource**：表示**嵌入到Prompt中的资源内容**。字段包括`type: "resource"`，以及一个`resource`对象，其类型为`TextResourceContents`或`BlobResourceContents`[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=LLM%20API,extra%3D%E2%80%9Dallow%E2%80%9D%29%20class)。`annotations`同样可选。EmbeddedResource用于在Prompt消息中内联一个资源的具体内容，保留资源的来源URI和类型信息，方便LLM和用户理解上下文。
    

**Resource 模块**提供数据资源的标识和读取机制，与Prompt内容紧密相关。每个Resource由唯一的URI标识，并有元数据如名称、描述、mimeType等[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=model_config%20%3D%20ConfigDict,None%20%3D%20None)。Resource并不直接包含数据，而是通过`resources/read`请求获取其内容[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=Reading%20resources)。Resource内容有两种形式：

* **TextResourceContents**：继承自ResourceContents，包含文本数据字段`text`[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)。用于表示可用UTF-8文本表示的资源内容（如代码、日志、文本文件）。
    
* **BlobResourceContents**：继承自ResourceContents，包含二进制数据字段`blob`（Base64编码的字符串）[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)。用于表示二进制资源内容（如图像、PDF、音频等）[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=Binary%20resources)。由于JSON不能直接传输二进制，需经Base64编码封装在字符串中[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=,%2F%2F%20Optional%20MIME%20type)。
    

**ResourceContents**基类提供通用字段：资源的`uri`和可选`mimeType`[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=ResourceContents,the%20binary%20data%20of%20the)。Server在返回资源内容时，会根据数据类型选择填充`text`或`blob`字段。JSON Schema中规定`ReadResourceResult.contents`是一个数组，可包含一个或多个内容项[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=BlobResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20params%3A%20NotificationParams)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=,%2F%2F%20Optional%20MIME%20type)。通常读取单一文件资源返回单个内容；若读取的是复合资源（如目录），可能返回多个子资源内容[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=)。

**Prompt 与 Resource 的关系**在于：Prompt模板可以引用资源URI作为参数，由Server读取资源后将内容嵌入Prompt消息，以EmbeddedResource或相应Content类型返回给客户端[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,text%2Fplain)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,n%20%20%20%20pass)。PromptMessage.content为EmbeddedResource时，内部包含ResourceContents（Text或Blob）以及原始URI和mimeType信息[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,python)。这样LLM接收到Prompt时，不仅看到原始内容（文本或base64数据），也保留了内容类型，有助于模型处理不同模态的数据。

**Prompt 与 Tool 模块的关系**：Prompt和Tool都可以产出类似的内容结构（TextContent/ImageContent/EmbeddedResource）。区别在于Prompt消息是Server预先定义的对话上下文，而Tool是在对话过程中由LLM触发调用，Server执行后返回结果。工具调用的返回也可能包含TextContent或EmbeddedResource列表[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/tools#:~:text=%40app,text%3Dstr%28result)。因此在实现SDK时，这部分内容类型处理可以通用。总的来说，Prompt模块通过Content类型将Resource数据融合到对话中，为LLM提供丰富的上下文。

## Prompt请求中资源嵌入的处理流程

当Prompt模板涉及嵌入资源（如要求传入文件URI、图像URI等参数）时，客户端和服务端需配合完成资源内容的获取与注入，具体流程如下：

1. **客户端请求Prompt并提供资源引用**：客户端通过`prompts/get`方法请求指定Prompt模板，并提供所需参数[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=To%20use%20a%20prompt%2C%20clients,request)。如果Prompt定义包含资源URI参数（例如参数名`fileUri`），客户端会将具体的URI填入请求参数中[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=%22name%22%3A%20%22analyze,fileUri)。请求JSON示例:
    
    ```json
    {
      "method": "prompts/get",
      "params": {
        "name": "analyze-project",
        "arguments": {
          "timeframe": "1h",
          "fileUri": "file:///path/to/code.py"
        }
      }
    }
    ```
    
    其中`fileUri`指向客户端希望Server嵌入的资源。Resource URI可以是Server支持的任意方案，例如本地文件、数据库、API等[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=Resources%20represent%20any%20kind%20of,This%20can%20include)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=,screen%3A%2F%2Flocalhost%2Fdisplay1)。
    
2. **服务端解析请求并获取资源内容**：Server接收到Prompt请求后，根据模板逻辑，识别需要嵌入的资源URI参数。Server此时会调用相应的资源读取机制获取内容：
    
    * 若Server已将该URI注册为Resource（在`resources/list`或模板中声明），可以直接调用内部函数或数据源获取内容。例如，Server可能有对应`logs://recent`或`file://`协议的处理函数。
        
    * Server可以直接在生成Prompt消息时读取文件或调用API获取数据。在MCP Python SDK中，如果资源函数返回`str`文本，会构造TextResourceContents；返回`bytes`则构造BlobResourceContents[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=import%20base64)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=return%20types)。Server确保在返回前将二进制内容Base64编码后填入`blob`字段[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=return%20types)。
        
    * Server也可使用标准的`resources/read`请求流程，即模拟客户端对自身发起资源读取RPC，再获取结果。但通常在Prompt实现中直接调用内部代码更方便。
        
    
    例如，Prompt模板“analyze-project”需要读取日志和代码文件[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,python)。Server可能执行如下伪代码：
    
    ```python
    logs = read_logs(timeframe)            # 得到日志文本
    code = open_file(file_uri)            # 读取文件二进制内容
    messages = [
        {"role": "user", "content": TextContent(text="Analyze these logs and code:")},
        {"role": "user", "content": EmbeddedResource(resource=TextResourceContents(uri="logs://recent?timeframe=1h", text=logs, mimeType="text/plain"))},
        {"role": "user", "content": EmbeddedResource(resource=TextResourceContents(uri=file_uri, text=code_text, mimeType="text/x-python"))}
    ]
    ```
    
    如上，日志属于纯文本，直接嵌入为TextResourceContents.text；代码虽然也是文本，也可作为TextResourceContents嵌入[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,python)。**若资源是二进制**（如图片、音频），Server需要将其读取为字节数组，并Base64编码为字符串，然后填入BlobResourceContents.blob字段[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)。MCP规范要求Binary资源内容以Base64字符串形式传输[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=,%2F%2F%20Optional%20MIME%20type)。
    
3. **服务端构造Prompt返回消息**：Server按Prompt定义，将静态模板内容和动态资源内容合成Prompt消息列表。每条消息的content根据数据类型填充：
    
    * 纯文本内容 -> TextContent。例如提示说明或用户输入模板文本。
        
    * 资源文本内容 -> EmbeddedResource，内含TextResourceContents（携带uri和text）[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D)。这种形式保留了资源来源信息，便于客户端/UI显示和模型参考。[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D)示例中，`mimeType: "text/plain"`标明嵌入的是纯文本。
        
    * 资源二进制内容 -> EmbeddedResource，内含BlobResourceContents（携带uri和base64数据blob）。同时mimeType注明类型（如`image/png`等）[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)。
        
    * 另一种选择：对于常见图像等，也可直接使用ImageContent而非EmbeddedResource。ImageContent也是包含base64数据和mimeType的结构[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextContent,%E2%80%9C%E2%80%9D%E2%80%9D%20annotations)。差别在于ImageContent没有资源URI元数据，只表示一段图像内容。**如果Prompt内容源自Resource**，倾向于EmbeddedResource以保留URI；如果是Prompt模板内置的固定图片，则可用ImageContent直接提供。
        
    
    Server封装好Prompt响应JSON后，通过MCP连接返回给客户端。例如返回内容结构如下（摘自官方示例）：
    
    ```json
    {
      "description": "Analyze project logs and code",
      "messages": [
        { "role": "user", "content": { "type": "text", "text": "Analyze these system logs and the code file for any issues:" } },
        { "role": "user", "content": { "type": "resource", "resource": {
            "uri": "logs://recent?timeframe=1h", 
            "text": "[2024-03-14 15:32:11] ERROR: Connection timeout...\n...", 
            "mimeType": "text/plain" } } },
        { "role": "user", "content": { "type": "resource", "resource": {
            "uri": "file:///path/to/code.py",
            "text": "def connect_to_service(timeout=30):\n    retries = 3\n    ...",
            "mimeType": "text/x-python" } } }
      ]
    }
    ```
    
    上例中，两段资源（日志和代码）均作为EmbeddedResource嵌入，`text`字段直接包含了文件内容[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,python)。如果其中有二进制内容，例如图像，则对应`resource`对象会使用`blob`字段承载base64数据，而非`text`[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)。
    
4. **客户端处理Prompt响应**：客户端收到Prompt响应后，需要提取消息列表供后续模型调用使用。其中资源内容的处理是关键：
    
    * **文本内容**：直接读取TextContent.text字符串即可使用。
        
    * **嵌入的文本资源**：EmbeddedResource内的TextResourceContents.text已经是可读文本，通常直接并入对话上下文传给模型（可能在提示中视为用户提供的资料片段）。
        
    * **嵌入的二进制资源**：EmbeddedResource内的BlobResourceContents.blob是base64字符串。客户端SDK应在调用模型前将其解码为原始二进制数据，以供模型使用（因为大多数LLM API需要原始文件数据或字节流）。例如，Python SDK在测试中验证了对blob的解码：[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=match%20at%20L12176%20base64.b64decode%28getattr%28bytes_result.contents%5B0%5D%2C%20,assert)中将`contents[0].blob` base64解码后比对原始bytes，确保传输正确。
        
    * **ImageContent/AudioContent**：类似地，`data`字段是base64字符串，客户端也需解码。[docs.rs](https://docs.rs/agentic/latest/agentic/mcp/struct.AudioContent.html#:~:text=%C2%A7%60data%3A%20Vec)表明音频内容在JSON中也是base64编码形式。在C++实现中，可使用Base64解码库，将ImageContent.data或AudioContent.data转为字节数组。
        
    * **Content元数据**：对于EmbeddedResource，content.resource内的`mimeType`可指导客户端如何处理数据。例如mime为`image/png`则解码bytes后可按PNG文件处理；mime为文本也可验证编码。URI信息主要用于显示或日志，无需传给模型但可辅助调试或UI提示来源。
        
    * **模型输入构建**：客户端通常需要根据Prompt消息构建LLM对话上下文。例如，将若干条PromptMessage作为系统或用户消息，最后可能还有真实用户提问（视具体应用）。如果Prompt消息序列的最后一条是用户角色内容（如上例两个resource都由用户角色给出），模型应接着生成assistant回应。因此客户端可能将这些Prompt消息直接传递给模型接口，让模型生成下一条assistant消息作为结果。
        
    
    值得注意的是，**MCP协议本身不直接执行LLM推理**，它负责提供上下文。实际调用模型（如调用OpenAI/Anthropic API或本地模型推理）通常由**MCP客户端（Host）**完成[modelcontextprotocol.io](https://modelcontextprotocol.io/introduction#:~:text=At%20its%20core%2C%20MCP%20follows,can%20connect%20to%20multiple%20servers)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/architecture#:~:text=MCP%20follows%20a%20client,where)。也就是说，在SDK中Prompt模块获取到消息后，需要与模型推理模块对接：
    
    * 简单情况下，客户端拿到Prompt返回的`messages`，再附加用户最终问题（如果Prompt不是完整对话而只是上下文），一起调用底层LLM API获取回答。
        
    * 如果Prompt已经包含完整的用户query和上下文（例如模板就是“一问一答”形式的对话开头），客户端直接将Prompt消息发送给模型让其续写assistant回答。
        
5. **模型生成结果及后续**：模型在得到Prompt提供的上下文后生成回答，通过客户端UI或应用反馈给用户。整个流程中，资源内容从Server安全获取并提供给模型，从而让LLM具备所需的外部知识或数据。MCP确保这一过程标准化：资源通过URI引用，内容以统一结构嵌入，上下文以JSON形式传输。
    

综上，Prompt请求中的资源嵌入流程包括：**客户端提供URI -> 服务端获取并编码内容 -> 内容随Prompt消息返回 -> 客户端解码使用**。这个过程中应特别关注Base64编码的正确处理和内容类型元数据的传递。

## 服务端资源获取与Base64编码处理

MCP对资源内容传输的约定影响了客户端和服务端SDK的实现：

* **服务端通过URI获取资源内容**：Server需要解析URI来定位资源。URI格式一般为`[protocol]://[host]/[path]`[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=)。具体协议和路径由Server实现决定，Host部分可选（例如`file:///path`省略host或用localhost；自定义协议如`logs://`可将参数编码在path或查询中）。对于**本地URI**（无host或host为`localhost`），Server直接在本地主机查找资源（如文件路径映射到文件系统）。对于**跨主机URI**（host为其他地址或协议为HTTP等外部资源），处理方式取决于Server实现的策略：
    
    * Server**可以支持远程抓取**：例如URI为`http://example.com/data.json`，Server若允许，需发起HTTP请求获取内容，然后包装成ResourceContents返回。这需要Server实现HTTP方案的解析和安全控制（防止SSRF等风险）。
        
    * Server**可能限制跨主机访问**：出于安全，Server可以只允许特定协议或host（如拒绝`file://`访问非本地主机路径）。在URI解析约束上，MCP的schema设定host非必需[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=list,%E2%80%9Cresources%2Fread%E2%80%9D%5D%20params%3A%20ReadResourceRequestParams%20class)并未强制检查，但Server可以在实现中验证。例如，仅当host为空或等于自身标识时才访问文件，否则返回错误。
        
    * **自定义协议的host参数**：有些协议用host部分传参数，例如`db://prod-db/records`，host=`prod-db`可能选择不同数据库源。这完全由Server开发者定义实现。**C++ SDK**需要提供灵活的URI解析接口，让开发者插件式地处理不同scheme和host。
        
* **资源内容编码**：Server读取到资源后，需根据类型进行编码：
    
    * 文本资源：确保是UTF-8文本，直接作为字符串赋值给TextResourceContents.text[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)。
        
    * 二进制资源：将字节数据用Base64编码为ASCII字符串，赋值给BlobResourceContents.blob[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)。同时设置正确的mimeType（如文件扩展或内容推断）。
        
    * 图像或多媒体资源：如果Server直接生成ImageContent/AudioContent返回，则同样需要Base64编码填充data字段[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextContent,%E2%80%9C%E2%80%9D%E2%80%9D%20annotations)。许多SDK内部提供了编码工具。例如Python SDK中使用`base64.b64encode(data).decode()`将bytes转字符串[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=import%20base64%20from%20pathlib%20import,Path)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=return%20types)。
        
    * Server返回的JSON中，一个资源内容示例如：[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=,%2F%2F%20Optional%20MIME%20type)：
        
        ```json
        {
          "uri": "file:///home/user/image.png",
          "mimeType": "image/png",
          "blob": "<base64字符串>"
        }
        ```
        
        客户端据此可识别该资源为PNG图片且blob字段有数据。Server也可选择EmbeddedResource封装整个结构[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=LLM%20API,extra%3D%E2%80%9Dallow%E2%80%9D%29%20class)。
        
* **多段内容的情况**：如前述，`resources/read`允许返回内容数组[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=)。Prompt嵌入时一般每条消息对应一个内容。如果Server返回多个内容段（如读取一个目录返回多个文件内容），客户端可能需要分别嵌入多条消息，或将它们串联为一个长文本。设计SDK时需考虑这种可能：内容列表迭代处理，每项用单独EmbeddedResource表示，更清晰地保留每个子资源的URI。
    
* **客户端对Base64的解码**：客户端SDK负责在将内容交给模型前进行适当解码转换：
    
    * 文本无需解码，但要注意确保文本编码正确无误。
        
    * Base64字符串需要解码为原始bytes。例如在Python中用`base64.b64decode()`, 在C++可使用Boost.Base64或OpenSSL库等解码[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=base64.b64decode%28getattr%28bytes_result.contents%5B0%5D%2C%20,assert)。
        
    * 解码后数据可以直接传递给模型SDK。例如OpenAI API需要Multipart上传文件，Claude API需要特殊接口等。C++实现可考虑封装ImageContent/AudioContent对象的方法，如`get_bytes()`返回解码后的`std::vector<uint8_t>`，方便调用者使用。
        
* **注意mimeType**：客户端获得mimeType后，一般不会改变其用途，但可作为验证。例如解码后可检查数据格式（magic number）是否匹配mime，或用于决定走哪个模型接口（如图像可能调用图像模型）。在指导C++实现时，可以将mimeType作为Content对象的属性保留，以备上层决策使用。
    

简而言之，服务端SDK需要提供**统一的资源读取与编码机制**，客户端SDK提供**透明的解码与数据封装**。C++实现时，可以设计ResourceReader接口处理各类URI，以及Content对象的encode/decode方法。要确保与其他语言SDK行为一致：**始终使用Base64编码传输二进制**[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=,%2F%2F%20Optional%20MIME%20type)、**文本与二进制内容分开表示**[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)。

## 本地 URI 与跨主机 URI 的差异处理

在MCP资源访问中，“本地URI”通常指Server本地主机或受控环境内的资源；“跨主机URI”指涉及远程主机或外部服务的资源定位。两者在解析和处理流程上有所不同：

* **本地URI（Local URI）**：这类URI直接映射Server可直接访问的数据源：
    
    * 典型例子：`file:///path/to/file`（本地文件系统路径）、`screen://localhost/display1`（本机屏幕截图设备）等[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=For%20example%3A)。这些URI的host部分为空或为`localhost`，表明资源位于Server所在主机。
        
    * 处理流程：Server可以无网络请求地直接访问。例如调用文件I/O读取文件；调用本地API获取屏幕截图。**解析**时通常跳过host或验证host是否为自身。
        
    * 安全性：访问本地资源需要Server有相应权限。C++实现中可结合操作系统API（如ifstream文件读取）。注意防止越权访问，如对file://路径进行沙箱限制（可限制只能访问配置的目录等）。
        
* **跨主机URI（Cross-host URI）**：这类URI指向Server外部的资源或服务：
    
    * 例子：`http://api.example.com/data`（远程HTTP资源）、`file://remotehost/shared/file`（网络共享文件），或者自定义如`db://otherdb/table1`（假设otherdb是另一台数据库主机）。
        
    * 处理流程：Server需要通过网络或远程连接获取。这可能涉及HTTP客户端、数据库连接等。**延迟和可靠性**也需考虑，远程请求可能较慢或失败需要超时处理。
        
    * 安全和权限：Server实现者应明确允许哪些外部访问。例如Whitelist特定域名或协议。默认情况下，一个MCP Server不会随意读取任意HTTP以避免安全风险，除非该Server的功能就是包装某HTTP API。在设计C++ SDK时，可以让开发者自行实现特定协议的ResourceHandler来处理跨主机访问，以便在代码中显式控制。
        
    * URI解析：如`file://remotehost/path`，一般的Server实现**不会**支持直接读取另一主机文件系统，除非有挂载或代理。因此可以约定：对于file://，非空host一律拒绝或忽略（仅限本地）；对于http/https，可以使用HTTP库获取。
        
* **URI host字段的作用**：MCP URI格式中host并不一定是网络主机，也可能用于逻辑命名。在自定义scheme下，host由Server解释。[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=The%20protocol%20and%20path%20structure,their%20own%20custom%20URI%20schemes)指出Server可定义自有URI scheme。因此host是否跨主机取决于实现：
    
    * 例如自定义`storage://us-east/mybucket/file1`，host=`us-east`也许只是区域标识，Server根据它去对应区域的存储服务抓取，但仍在Server权限范围内。
        
    * 又如`postgres://dbserver/mydb/table`, host=`dbserver`表示数据库实例名，Server可能有配置映射`dbserver`到实际连接参数。
        
    * 这些情况下，**host不一定真的跨物理主机**，而是作为资源定位符一部分。C++ SDK应提供灵活性，让不同scheme自定义解析host含义。
        
* **差异总结**：本地URI通常**同步、本地读取**，实现简单速度快；跨主机URI可能**需要异步或耗时IO**，实现复杂需考虑网络错误。对于Prompt模块而言，如果资源参数给出跨主机URI且Server实现支持，流程与本地类似但在Server获取内容阶段延迟更高。**SDK实现上**应保证:
    
    * 提供配置或插件，让开发者启用/禁用某些协议的跨主机访问。
        
    * 在ResourceContents中保持原URI不变，使客户端知晓内容来源。
        
    * 对于跨主机内容，可能在ResourceContents.annotations中标记一些信息（priority之类，可选[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=The%20size%20of%20the%20raw,BaseModel%29%3A%20%E2%80%9C%E2%80%9D%E2%80%9DA)），不过一般无需特殊区分。
        

简单来说，**本地URI**在Server内部解析处理，**跨主机URI**可能引入网络交互。C++ SDK的Resource模块需设计成**可扩展的URI处理框架**，同时在默认实现中对高危跨主机访问采取保守策略。

## Prompt嵌入资源的端到端调用流程

结合以上要点，下面以端到端视角梳理一次完整的Prompt调用流程，其中嵌入了资源内容：

1. **客户端（C++ SDK 使用者）初始化**：通过SDK提供的客户端类连接一个MCP Server（可能通过stdio或SSE）[developer.aliyun.com](https://developer.aliyun.com/article/1662194#:~:text=MCP%E6%94%AF%E6%8C%81%E4%B8%A4%E7%A7%8D%E4%BC%A0%E8%BE%93%E6%96%B9%E5%BC%8F%EF%BC%9A%E6%A0%87%E5%87%86%E8%BE%93%E5%85%A5%E8%BE%93%E5%87%BA%EF%BC%88stdio%EF%BC%89%E5%92%8C%E5%9F%BA%E4%BA%8E%20HTTP%20%E7%9A%84%E6%9C%8D%E5%8A%A1%E5%99%A8%E6%8E%A8%E9%80%81%E4%BA%8B%E4%BB%B6%EF%BC%88SSE%EF%BC%89%E3%80%82)。确保已经获取可用Prompt列表（可调用`prompts/list`获取Server支持的Prompt名称等）。
    
2. **用户选择并触发Prompt**：假设用户选择了某Prompt（如“分析文件”），并提供了所需参数（如文件URI、分析选项等）。客户端代码调用SDK的`getPrompt(name, params)`接口，底层组装`prompts/get`请求发送至Server。
    
3. **服务端生成Prompt消息**：Server Prompt模块收到请求后，匹配到相应Prompt模板。Server代码（可能用C++ SDK服务端部分实现）按照模板定义执行：
    
    * 验证参数完整性，如果有缺 required参数未提供则返回错误。
        
    * 使用参数中的URI读取资源：调用Resource模块接口，如`ResourceManager::read(uri)`。后者找到对应ResourceHandler并返回内容（包含Base64编码）[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=return%20types)。
        
    * 构造Prompt消息列表：将模板中的静态文本包装为TextContent，将获取的资源内容根据类型包装为EmbeddedResource或ImageContent等。
        
    * 将所有PromptMessage放入响应，发送回客户端。C++服务端SDK应序列化对象为符合schema的JSON（注意类型字段`type`和嵌套结构正确）。
        
4. **客户端处理Prompt响应**：C++客户端SDK接收到JSON后反序列化为PromptResult对象，包含Prompt描述和消息列表：
    
    * SDK应提供PromptMessage类型（含role和Content子类的variant）。根据`type`字段将content解析成TextContent、ImageContent或EmbeddedResource对象。
        
    * 对于EmbeddedResource，内部还需判断是TextResourceContents还是BlobResourceContents（看是text字段存在还是blob存在）。
        
    * 全部解析后，SDK将Prompt消息传递给调用方代码。**此时资源内容仍以base64字符串形式存在于对象中**（对于binary内容）。
        
5. **客户端调用LLM**：客户端应用现在需要将Prompt提供的上下文发送给实际的大模型推理服务：
    
    * 如果使用OpenAI/Anthropic API：需要将消息转换为相应格式。例如OpenAI需要messages数组，每项有role和content；content若有图像，要按API要求发送文件。此处SDK可辅助：
        
        * 提供方法将PromptMessage转换为标准对话消息（文本直接用字符串；EmbeddedResource如图像则输出特殊标记或者提供原始数据）。
            
        * **关键**：对于ImageContent/AudioContent等，SDK应当**解码base64**得到二进制，并提供给调用方。例如提供一个`content.getDataBytes()`方法获取`std::string`或`vector<uint8_t>`。
            
        * C++客户端可能集成具体模型SDK，也可能将数据返回让上层处理。如果SDK要封装与某模型API的集成，需要知道如何传二进制（这可能超出MCP SDK职责）。
            
    * 如果模型在本地（例如用户将LLM作为函数调用）：SDK可以直接将资源bytes传入模型接口。
        
    * 在这个阶段，Prompt消息中的角色和顺序决定了如何调用模型。常见情况是：Prompt提供了一系列系统/用户消息作为上下文，**模型需要生成下一条assistant消息**。因此客户端会触发模型的推理，传入Prompt消息作为上下文。例如“用户: 请分析X; 用户: [文档内容]”，模型需要给出assistant答复。
        
6. **LLM生成回答**：模型根据Prompt和资源上下文生成结果。客户端获取该结果后，可能封装为MCP对话的一部分或直接显示给用户。由于Prompt本身是一次性上下文，通常这一步不需要再经由MCP Server，除非Server还有后续交互（例如Prompt中建议调用工具，LLM可能触发Tools调用的流程，那又经过MCP交互，但那是另一话题）。
    
7. **后续交互**：Prompt流程完成后，如果用户继续对话，客户端可以继续直接与模型交互或再次调用MCP其它能力。**MCP Session**可以视Prompt内容为对话历史一部分，也可以每次使用Prompt产生全新对话上下文。这取决于Host应用逻辑。
    

整个端到端过程中，**Prompt模块起到了桥梁作用**：让本地或后端的数据通过标准化步骤送达LLM。对于C++ SDK实现者而言，需保证每一步的数据结构清晰转换、不丢失信息：

* URI、mimeType等在需要时可访问，用于调试或日志。
    
* Base64编码解码正确无误，确保LLM拿到有效数据（如图像不损坏，文本不乱码）。
    
* Content类型的多态处理，在编译期尽量通过类型系统明确区分，减少运行时错误。例如可用`std::variant<TextContent, ImageContent, EmbeddedResource>`表示PromptMessage.content，根据`type`字段解析赋值。
    

## Python SDK与Java SDK实现差异对比

MCP已有官方的 Python 和 Java SDK 实现，这两种语言的设计为C++版提供了有益参考：

* **类型定义方式**：Python SDK使用Pydantic模型定义了Prompt及内容相关类。例如Prompt、PromptMessage、TextContent等作为Pydantic BaseModel，利用`Literal[...]`和Union来区分类型[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=ConfigDict,to%20a%20prompts%2Fget%20request%20from)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextContent,to%20or%20received%20from%20an)。这使JSON反序列化自动将`type="text"`映射到TextContent类等。Java SDK则可能使用Jackson或手写解析，将JSON中的`type`字段用于多态反序列化。C++没有直观的反射机制，可以采用**显式枚举或tag**实现，如定义ContentType枚举（TEXT/IMAGE/RESOURCE），再据此存放不同结构，或使用variant模板。如[hkr04/cpp-mcp 项目]中采用了struct继承+枚举的方法来表示Content类型（从Go SDK实现可推测[pkg.go.dev](https://pkg.go.dev/github.com/SetiabudiResearch/mcp-go-sdk/pkg/mcp/protocol#:~:text=...%20pkg.go.dev%20%20MCP,)）。
    
* **资源读取实现**：Python SDK通过装饰器和类型推断实现了资源函数返回值到ResourceContents的转换。例如若资源函数返回`bytes`，SDK将其作为BlobResourceContents处理[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=return%20types)；返回字符串则作为TextResourceContents[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=match%20at%20L12131%20base64_string%20%3D,8)。Java SDK（Spring AI协作实现）可能采用注解或接口让开发者提供资源内容，并要求明确返回类型或封装类（如返回一个ResourceContents对象）。C++可借鉴**策略模式**：每种协议对应一个处理类，提供`read(uri) -> ResourceContents`。C++缺少运行时类型推断返回值，可让开发者直接构造并返回TextResourceContents或BlobResourceContents对象，避免歧义。
    
* **Base64处理**：两种SDK都借助标准库处理。Python用`base64`模块编码/解码[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=import%20base64%20from%20pathlib%20import,Path)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=base64.b64decode%28getattr%28bytes_result.contents%5B0%5D%2C%20,assert)。Java可能使用Java内置Base64类。C++可使用Boost.Base64或其他轻量实现。在设计上，可以将Base64处理封装在ResourceContents的序列化/反序列化过程中，开发者不需手工调用。比如当Server读取文件返回字节数组时，SDK自动编码填入JSON；当Client解析到BlobResourceContents时，SDK自动解码填充一个内部`std::string bytes`，同时也保留原始base64字符串以防需要。这样**模型调用阶段**可直接取bytes。
    
* **资源与Prompt交互经验**：实际实现中要注意以下几点（从社区经验和SDK演进来看）：
    
    * **大文件分段**：如果资源非常大，全部嵌入Prompt可能超出上下文窗口。[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=)提到Server可返回多项内容（如目录列表），但对大文件没有拆分标准。Python/Java SDK当前可能先整体传输，由上层决定截断。C++实现可考虑提供**文件内容分块读取**接口（但仍需协议支持传多块顺序）。
        
    * **Prompt注入与安全**：有博客指出MCP Prompt在安全方面要防范Prompt Injection[medium.com](https://medium.com/@itsuki.enjoy/model-context-protocol-mcp-authorization-the-new-rs-and-as-separation-4acfd700db14#:~:text=Model%20Context%20Protocol%20,the%20discussions%20regarding%20to)。SDK实现应提示开发者：嵌入用户提供的资源内容时，小心其中包含的模型指令。不过这更多是应用层的问题，SDK本身只传输数据。可以利用Annotations的audience字段，将资源内容标记给LLM或仅供用户查看[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=%E2%80%9C%E2%80%9D%E2%80%9DA%20description%20of%20what%20this,BaseModel%29%3A%20%E2%80%9C%E2%80%9D%E2%80%9DA)。
        
    * **多语言差异**：Java在强类型下，可能每种Content都有独立类并实现一个接口。Python灵活但可能隐藏错误，比如如果Server误将二进制当文本放入TextResourceContents，会导致LLM看到的是base64字符串而非真正解码的数据。这在测试中已有体现：若将图像base64作为text返回，Client仍会拿到base64字符串[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=base64_string%20%3D%20base64.b64encode%28image_bytes%29.decode%28%22utf)。因此C++实现应**严格区分**，避免混淆。推荐让开发者明确调用不同接口，如`Resource::readText`返回文本，`Resource::readBinary`返回二进制，以防错误用法。
        
    * **性能和内存**：C++关注内存管理，大文件base64编码会使数据膨胀约33%。SDK应尽量使用流式处理避免峰值内存。Python可能直接读入内存后编码，Java类似。C++可以利用std::ifstream逐块读文件并Base64流式编码输出JSON，减少内存拷贝。这是实现层面的优化点。
        
* **调用模型接口**：Python和Java SDK本身不直接包含LLM调用逻辑（MCP不限定LLM厂商）。但一些Client示例展示了如何将Prompt结果用于OpenAI API等。例如LangChain4j对MCP的支持，将MCP返回的内容转换成其内部对话格式[glaforge.dev](https://glaforge.dev/llms-full.txt#:~:text=full.txt%20,file%2C%20the%20whole%20Micronaut%20single)。C++ SDK可提供类似**适配层**：例如转换PromptMessage列表为OpenAI ChatCompletion API的JSON。这样SDK用户更方便地衔接模型调用。当然，这部分可作为附属工具模块，不强耦合在MCP核心协议实现里。
    

总的来说，Python SDK体现了**易用性**（自动类型处理、装饰器简化开发）、Java SDK体现了**严谨性**（类型安全、结合框架）。C++实现应在两者间权衡：提供清晰类型结构和必要的自动处理，同时给予开发者控制底层数据的能力。

## 结论与设计建议

通过上述分析，MCP Prompt模块与Resource、Content模块关系紧密，完整实现需要处理好多类型内容的序列化、资源远程访问及数据编码转换。在C++ SDK设计Prompt模块时，建议重点关注：

* **清晰的类型体系**：定义Prompt、PromptMessage、Content（Text/Image/Audio/EmbeddedResource）、ResourceContents等结构体或类，利用C++类型系统区分不同内容，避免混淆。不妨参考MCP JSON Schema[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextContent,to%20or%20received%20from%20an)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=LLM%20API,extra%3D%E2%80%9Dallow%E2%80%9D%29%20class)直接映射为C++类。
    
* **完善的编码解码支持**：集成Base64编码库，封装在ResourceContents的getter/setter中。确保Server侧自动把bytes编码，Client侧拿到时容易解码成字节缓冲。
    
* **URI处理扩展点**：提供注册机制让开发者为不同scheme编写读取器（例如FileReader, HttpReader）。默认实现安全地处理常用`file`协议，并对未知协议给出错误或留待用户实现。
    
* **参数与内容拼装**：Prompt模板的实现可能在Server侧以代码编写（不像Python有动态模板语言）。C++可让开发者在Prompt处理器中手动构造PromptMessage列表。为了便利，可提供类似Python那样的helper，例如`make_text_message(role, string)`，`make_resource_message(role, uri, data, mimeType)`等函数封装对象创建。
    
* **对标SDK行为**：遵循官方SDK在边界条件下的处理方式，以确保兼容。例如当资源为文本时，一定使用TextResourceContents.text而不放blob；当返回EmbeddedResource时，一定带上正确的mimeType。[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)和[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=,%2F%2F%20Optional%20MIME%20type)等处的规范需严格实现。
    

通过深度理解Prompt与Resource内容交互的机制，我们在C++实现中就能既保证与规范一致，又发挥C++性能优势，为开发者提供可靠的MCP客户端/服务端SDK Prompt模块。以上调研结论和对比经验，将为C++ SDK的设计和开发提供有力指导。

**参考资料：**

* Model Context Protocol 官方文档[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=Create%20reusable%20prompt%20templates%20and,workflows)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,%7D)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/prompts#:~:text=,python)[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/resources#:~:text=,%2F%2F%20Optional%20MIME%20type)等
    
* MCP JSON模式定义（2025-03-26）[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextContent,to%20or%20received%20from%20an)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=LLM%20API,extra%3D%E2%80%9Dallow%E2%80%9D%29%20class)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextResourceContents,%E2%80%9Cnotifications%2Fresources%2Flist_changed%E2%80%9D%5D%20%5D)
    
* MCP Python SDK 实现细节[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=TextContent,%E2%80%9C%E2%80%9D%E2%80%9D%20annotations)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=return%20types)[streetsdigital.com](https://streetsdigital.com/2025/04/04/mcp-python-sdk/#:~:text=match%20at%20L12176%20base64.b64decode%28getattr%28bytes_result.contents%5B0%5D%2C%20,assert)
    
* MCP Java SDK (Spring AI) 实现思路和社区示例[modelcontextprotocol.io](https://modelcontextprotocol.io/docs/concepts/tools#:~:text=%40app,text%3Dstr%28result)[glaforge.dev](https://glaforge.dev/llms-full.txt#:~:text=full.txt%20,file%2C%20the%20whole%20Micronaut%20single)
    
* C++ MCP示例工程与第三方SDK参考 (hkr04/cpp-mcp 等)[pkg.go.dev](https://pkg.go.dev/github.com/SetiabudiResearch/mcp-go-sdk/pkg/mcp/protocol#:~:text=...%20pkg.go.dev%20%20MCP,)