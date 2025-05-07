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
   
请优先回答我的目标。然后可以对我的资源进行合理的建议和开发步骤，或者设计开发。目录和文件的调整等，还有增加，删除，或者时命名。

* * *

**ChatGPT:**

为了更好地帮你回答关于 C++ 实现 MCP 中 prompt 模块客户端和服务端的目标，请确认以下几点：

1. 你希望客户端和服务端通过什么协议通信？例如 HTTP/REST、gRPC、WebSocket 等？
    
2. prompt 模块是否只包含 prompt 本体（如 text、metadata），还是也涉及 resource 嵌入、引用、chunk 等子结构？
    
3. 你的 C++ 项目计划支持同步、异步还是两者通信方式？
    
4. 有无计划支持持久化 prompt 数据（如存储到数据库）或缓存？
    
5. 项目是计划部署为库、服务进程，还是支持 CLI 工具等接口？
    

确认这些可以帮助我提供更准确的模块拆分建议、接口设计和开发步骤规划。

* * *

**You:**


1. 客户端和服务端的通信方式：如果 不依赖于 boost 库的 asio 可以的话，使用这个。其次优先使用 grpc。
2. prompt 模块目前不止包含 prompt 本体，应该根据 mcp 协议包含 resource 嵌入，引用等。当然也可以只包含 prompt 本体吧，这个看 mcp 协议怎么做。你可以设计。
3. c++ 项目同步，异步都支持最好。
4. 支持持久缓存，可以先存内存。后续我们在存数据库。先跑通整个流程。
5. sdk 是库，服务或者客端端是进程，或者是某一个进程的一部分。

* * *

**ChatGPT:**

明白了。我将深入分析 MCP 协议中 prompt 模块涉及的结构、资源嵌入和引用机制，并明确客户端与服务端在 C++ SDK 中应支持的功能和接口（同步与异步）。此外，我会参考已有 Python 与 Java SDK 的设计方式，提出适合你当前 C++ 项目的合理结构调整建议、模块划分、文件命名和资源组织方式。  
我会尽快整理结果与你分享。

* * *

**ChatGPT:**

# 设计 MCP Prompt 模块的 C++ 客户端与服务端 SDK

## 引言 (Introduction)

Model Context Protocol（**MCP**）是由 Anthropic 开源的一种标准协议，旨在让大型语言模型（LLM）与各种本地或远程数据源连接[medium.com](https://medium.com/@vincentlambert0/mcp-model-context-protocol-the-open-source-protocol-revolutionizing-ai-integration-25a7a0c2db21#:~:text=MCP%20is%20an%20open%20standard,their%20model%20providers%20or%20tools)。其中 **Prompt 模块** 提供了一种可重用的 **提示模板**（prompt template）机制，用于定义标准化的交互模式和工作流程[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=values%2C%20standardizing%20the%20way%20LLMs,structured%20and%20manageable%20AI%20applications)。通过 Prompt 模块，开发者可以预先定义多步对话模板，支持在不同应用间共享和版本化，从而构建结构化、可维护的 AI 应用[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=values%2C%20standardizing%20the%20way%20LLMs,structured%20and%20manageable%20AI%20applications)。在 Prompt 模块中，提示可以包含文本、图像等多模态内容，甚至可以嵌入资源内容，使 LLM 能获取更丰富的上下文信息[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=LLMs%20and%20external%20systems,of%20these%20are)。

本文基于 **2025-03-26** 版本的 MCP JSON Schema[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=,nresources%20from%20the%20MCP%20server)以及 Python SDK 和 Java SDK 的实现方式，分析 Prompt 模块涉及的主要数据结构、资源嵌入与引用逻辑，并据此设计 C++ 语言的 Prompt 模块客户端和服务端 SDK 接口与功能。我们将介绍 Prompt 模块所依赖的其他结构和资源类型，阐述 Prompt 中资源嵌入与引用的处理流程，包括资源提取、管理和缓存策略，并分别给出客户端与服务端应提供的接口设计（涵盖同步与异步用法）。随后，我们结合现有的 C++ 项目结构，提出模块划分、JSON 绑定代码优化、文件命名组织改进的建议，并给出后续迭代计划与优先级建议。最后，通过**小结 (Summary)**总结设计要点。

## Prompt 模块的数据结构与依赖 (Data Structures and Dependencies)

Prompt 模块定义了一系列数据结构，用于描述提示模板及其内容。关键结构包括 **Prompt**（提示模板定义）、**PromptArgument**（提示参数）、**PromptMessage**（提示消息）以及各种 **Content** 类型（文本、图像、音频或嵌入资源）和 **Resource**（资源元数据）。下面详细介绍这些结构及其相互关系。

### Prompt 与 PromptArgument

**Prompt** 表示服务器提供的一个提示模板定义[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。其主要字段有：

* `name`：提示模板名称（字符串，必须字段）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `description`：提示模板的描述（可选，用于向用户解释用途）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `arguments`：参数列表，描述此 prompt 可接受的参数及其信息[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    

每个 prompt 可定义零个或多个参数，即 **PromptArgument** 列表。**PromptArgument** 描述了单个参数的名称、说明以及是否必填等属性[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)：

* `name`：参数名称（字符串，必须）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `description`：对该参数的说明（字符串，可选）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `required`：该参数是否必需（布尔值，可选，默认为非必需）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    

**提示模板依赖结构**：Prompt 结构本身相对简单，但它依赖 PromptArgument 定义参数。当客户端请求某个 Prompt 的实例时，可以提供这些参数的值以生成最终提示内容。Prompt 的列表和获取由专门的请求/响应结构管理（下文“客户端与服务端接口”将详述）。

### PromptMessage 与内容类型 (Content Types)

**PromptMessage** 表示 prompt 模板中的一条对话消息[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。它类似于 LLM 对话中的一条消息（如用户消息或助手回复），包括两个关键字段：

* `role`：消息角色，使用 **Role** 定义（例如 `"user"` 用户，`"assistant"` 助手 等）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `content`：消息内容，可以是多种类型之一[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    

MCP 支持多模态的内容类型，以丰富 Prompt 的表达[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=LLMs%20and%20external%20systems,of%20these%20are)：

* **TextContent**：文本内容（包括纯文本或Markdown）[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=LLMs%20and%20external%20systems,of%20these%20are)。包含字段 `type: "text"` 和实际文本字符串 `text`[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* **ImageContent**：图像内容，允许在 Prompt 中传递图片（通常以 base64 或 URI 表示）[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=LLMs%20and%20external%20systems,of%20these%20are)。包含 `type: "image"` 及图像数据或引用（具体结构在 JSON Schema 中定义）。
    
* **AudioContent**：音频内容，允许传递音频片段（结构类似 ImageContent，有 `type: "audio"`）。
    
* **EmbeddedResource**：嵌入资源内容（详见下节），用于在 Prompt 中内嵌服务器可用的资源数据[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    

在 JSON Schema 中，PromptMessage 的 `content` 定义为上述几种 content 类型的 **联合 (anyOf)**[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。这意味着每条 Prompt 消息的内容可以是文本、图像、音频，或一个嵌入的资源引用。[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。例如：

```json
"PromptMessage": {
  "properties": {
    "role": { "$ref": "#/definitions/Role" },
    "content": {
      "anyOf": [
        { "$ref": "#/definitions/TextContent" },
        { "$ref": "#/definitions/ImageContent" },
        { "$ref": "#/definitions/AudioContent" },
        { "$ref": "#/definitions/EmbeddedResource" }
      ]
    }
  },
  "required": ["role", "content"]
}
```

如上所示，PromptMessage 可包含文本或多媒体内容。如果需要在 Prompt 中引用外部资源（如文件内容），则使用 **EmbeddedResource** 类型承载。**Role** 则限定了消息扮演的身份（通常为 `"user"` 或 `"assistant"` 等字符串）。

### EmbeddedResource 与 Resource 资源结构

**EmbeddedResource** 表示嵌入到 Prompt 消息中的资源内容[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。它允许将服务器可访问的资源直接提供给 LLM 模型。例如，如果 Prompt 模板需要附带一段文件内容，服务器可以将文件作为 EmbeddedResource 附加在 PromptMessage 中。

EmbeddedResource 具有以下主要字段[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)：

* `type`: 固定为 `"resource"`（标识这是资源类型内容）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `resource`: 资源的具体内容，格式视资源类型而定[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
    * 如果资源是文本数据，则为 **TextResourceContents**[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)；
        
    * 如果资源是二进制数据（如图像、PDF），则为 **BlobResourceContents**[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
        

**TextResourceContents** 结构包含：

* `text`：资源的文本内容[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `uri`：资源的URI标识[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `mimeType`：资源MIME类型（如`text/plain`，可选）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    

**BlobResourceContents** 结构包含：

* `blob`：资源的二进制内容（经 base64 编码的字符串）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `uri`：资源URI标识[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `mimeType`：资源MIME类型（如`application/pdf`等）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    

注意，无论文本或二进制资源，都会包含一个 `uri` 字段，用于标识该资源的位置或ID[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。在嵌入资源时，服务器通常会同时提供内容本身和 URI 引用。**URI** 可被看作资源的唯一标识符，客户端可以利用它进行缓存或展示引用。

除了嵌入资源内容外，MCP 还定义了 **Resource** 类型，用于描述服务器可读取的已知资源（主要作为目录/元数据用途）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。Resource 包含：

* `name`：资源的人类可读名称[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `description`：资源代表内容的描述（可用作提示模型的上下文提示）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `uri`：资源的URI标识（必须字段）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `mimeType`：资源类型，如`text/plain`、`image/png`等[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `size`：资源内容大小（字节数，可选，用于估计对话上下文消耗等）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `annotations`：附加的注解信息（可选，用于UI或其他用途）。
    

Resource 列表可以由服务器提供，让客户端（或LLM代理）了解可用的资源及其基本信息。**Prompt 模块与 Resource 模块关系密切**：Prompt 可以嵌入 Resource 内容，而 Resource 列表由服务器管理。Resource 的元数据（如描述）可帮助 LLM 理解资源意义[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。例如，如果有一个名为“user_guide.txt”的资源，描述为“用户指南文档”，LLM 通过该描述可大致了解其内容，从而决定是否请求嵌入。

**总结**：Prompt 模块的数据结构建立在 MCP 核心的 JSON-RPC 消息规范之上。Prompt/PromptMessage 定义模板和对话消息，Content 和 EmbeddedResource 定义了消息中可包含的内容类型，而 Resource 则提供被引用资源的描述。下面将探讨 Prompt 模块如何处理资源的嵌入与引用。

## Prompt 模块中的资源嵌入与引用逻辑 (Resource Embedding and Reference Handling)

Prompt 模块的亮点之一是支持**资源嵌入**，即在提示内容中直接包含外部资源的数据。这使 LLM 在生成回答时可以利用上下文数据，如文件内容、图片等，而无需自行检索。实现这一能力需要服务器和客户端协调处理资源的提取、管理和缓存。下面详述 Prompt 模块的资源嵌入/引用流程和设计考虑。

### 资源嵌入的场景与流程

典型的资源嵌入场景是：**Prompt 模板需要使用某个资源的数据**。例如，一个 Prompt 模板要求模型根据用户指定的文件内容做总结。流程如下：

1. **用户发起 Prompt 调用**，指定 Prompt 名称和所需参数（其中可能包含一个资源标识，例如文件路径或 URI）。
    
2. **服务端 PromptService 接收到请求**后，从 PromptStore 中检索对应的 Prompt 模板定义，包括参数信息和模板内容。
    
3. **填充参数**：对于一般文本参数，直接替换模板中的占位符即可（通过 TemplateEngine，实现字符串插值）。对于**涉及资源的参数**，服务端需要提取该资源的内容：
    
    * 如果参数值是资源的URI（例如 `"file://path/to/file.txt"`），服务器据此读取文件内容。
        
    * 如果 Prompt 模板本身预先关联了某资源（比如模板固定引用某配置文件），服务器也需要访问该资源。
        
4. **组装 Prompt 消息**：对于提取的资源内容，服务器**不会简单地将其作为普通文本插入**，而是构造一个 EmbeddedResource 对象[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。具体步骤：
    
    * 判断资源类型（文本或二进制）并封装为相应的 TextResourceContents 或 BlobResourceContents，包括内容和 MIME 类型等。
        
    * 设置 EmbeddedResource 的 `uri` 字段为资源的标识符，以便客户端识别该资源。[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)
        
    * 将 EmbeddedResource 作为 PromptMessage.content 放入 Prompt 的消息列表中。
        
    * 例如，Prompt 模板可能定义了一条 user 消息，其内容占位符需要替换成一个文件内容。在 C++ 服务端实现中，我们会创建：
        
        ```cpp
        PromptMessage msg;
        msg.role = Role::User;
        msg.content = Content::EmbeddedResource({
            .text = file_text, 
            .uri = "file://path/to/file.txt", 
            .mimeType = "text/plain"
        });
        ```
        
        这样生成的 PromptMessage 包含文件内容作为嵌入资源。
        
5. **返回 Prompt 内容**：服务端将包含 EmbeddedResource 的 Prompt 消息列表打包在 GetPromptResult 中返回给客户端[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。客户端收到后，可以将嵌入资源内容提供给 LLM 模型作为上下文（例如，在提示中附上文件全文），或者在UI中以特殊方式显示（比如文件卡片形式）。
    

这种机制使**LLM 能直接利用资源的实际内容**，而不仅仅是一个引用。正如 MCP 规范所述，具体如何呈现嵌入资源内容可由客户端决定，以便更好地服务 LLM 或用户[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。例如，对于较大的文档，客户端也许会截断或分页显示给用户查看，但对于模型则可能全量提供文本。总之，**EmbeddedResource 提供了一种分层次传递信息的方式**，资源可以相互引用，形成信息层次结构，供 LLM 交互[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=,that%20LLMs%20can%20interact%20with)。

### 资源引用与缓存管理

由于资源内容可能较大或频繁使用，良好的**资源管理与缓存策略**至关重要。Prompt 模块在资源嵌入过程中需考虑以下几点：

* **权限与可访问性**：服务端在读取资源前，应确认该资源受支持且允许访问。MCP 通过 URI 方案限定可访问的资源范围（例如 `file://` 表示本地文件），服务器实现中应对不可信的URI予以过滤。
    
* **资源提取效率**：对于本地文件或数据库查询，在每次 Prompt 请求时重新读取可能产生性能开销。可以引入缓存来提高效率。
    
* **缓存粒度**：建议针对资源内容建立缓存，以 URI 作为键。初期可采用**内存缓存** (in-memory cache)，将最近使用或经常使用的资源内容暂存，避免重复读取磁盘或网络。[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=,structured%20and%20manageable%20AI%20applications)[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=,that%20LLMs%20can%20interact%20with)
    
* **缓存刷新失效**：由于初期不引入持久化数据库，内存缓存应有基本的大小管理和失效策略。例如，可采用 LRU (Least Recently Used) 算法限制缓存大小，当超过阈值时淘汰最久未使用的条目。也可根据资源的修改时间，判定缓存内容是否需要更新（例如文件有更新则刷新缓存）。
    
* **URI 标识唯一性**：利用 EmbeddedResource 的 `uri` 字段，使客户端缓存成为可能。如果客户端多次请求同一资源内容，且 `uri` 相同，客户端可识别重复并选择使用缓存内容而不重复加载。这也要求服务器对相同资源始终提供一致的 URI 标识（例如避免同一文件使用不同别名 URI）。
    

**示例：**  
在 C++ 服务端实现中，可以设计一个 `ResourceManager`（或在 ResourceService 中）维护 `std::unordered_map<std::string, ResourceData>` 作为缓存。每次读取文件前，检查缓存：

```cpp
auto it = cache.find(resource_uri);
if(it != cache.end()) {
    ResourceData &data = it->second;
    if(!data.isExpired()) {
        // 直接使用缓存的数据
        content = data.content;
    } else {
        content = readFile(resource_uri);
        data.update(content);
    }
} else {
    content = readFile(resource_uri);
    cache.emplace(resource_uri, ResourceData(content));
}
```

其中 `ResourceData` 可包含实际内容、时间戳等信息。初期实现中，可以简单地缓存所有读取过的资源，并提供清除或容量限制功能。

总之，Prompt 模块对资源的处理遵循**提取-封装-缓存**的逻辑：提取真实内容，封装为 EmbeddedResource 返回，并在两端利用 URI 进行缓存和复用。接下来，我们基于以上机制，设计 C++ Prompt 模块在客户端和服务端的接口和功能。

## 客户端 SDK 接口设计 (Client SDK Interface)

MCP 客户端通常由主机应用集成，用于与 MCP 服务端通信[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=,the%20ecosystem%20and%20expose%20critical)。针对 Prompt 模块，客户端 SDK 需要提供便捷的方法来获取服务器支持的 prompt 列表以及特定 prompt 的内容，供上层应用（例如聊天界面或AI代理）使用。下面设计 C++ 客户端接口，包括同步和异步形式。

### Prompt 列表获取 (List Prompts)

客户端应提供方法列出服务端的所有提示模板。例如：

```cpp
class PromptClient {
public:
    // 同步获取Prompt列表
    std::vector<Prompt> listPrompts();
    // 异步获取Prompt列表
    std::future<std::vector<Prompt>> listPromptsAsync();
    // ... 其他方法 ...
};
```

调用 `listPrompts()` 将触发向服务端发送 `prompts/list` 请求，取得可用 Prompt 列表[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。服务端响应包含一个 `prompts` 数组，内含每个 Prompt 的 name、description、arguments 列表等[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。SDK 内部会将该响应解析为 `std::vector<Prompt>` 对象返回给调用者。

* **同步接口**：`listPrompts()` 封装底层通信，线程阻塞等待结果返回。
    
* **异步接口**：`listPromptsAsync()` 立即返回一个 `std::future`，允许调用者随后获取结果或通过回调/等待完成，避免阻塞当前线程。
    

**使用示例：**

```cpp
PromptClient promptClient(connection);
// 同步调用
std::vector<Prompt> prompts = promptClient.listPrompts();
for(const auto& prm : prompts) {
    std::cout << prm.name << " - " << prm.description << std::endl;
}
// 异步调用
auto futurePrompts = promptClient.listPromptsAsync();
// ... do other work ...
if(futurePrompts.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
    auto promptsList = futurePrompts.get();
    // process promptsList
}
```

### Prompt 获取与参数传递 (Get Prompt)

客户端需要根据 Prompt 名称获取具体的 Prompt 实例内容。接口设计如下：

```cpp
class PromptClient {
public:
    PromptContent getPrompt(const std::string& name, 
                             const std::map<std::string, std::string>& arguments = {});
    std::future<PromptContent> getPromptAsync(const std::string& name, 
                             const std::map<std::string, std::string>& arguments = {});
    // ...
};
```

这里，`PromptContent` 可以是一个自定义类型，包含 Prompt 的描述和消息列表。例如：

```cpp
struct PromptContent {
    std::string description;
    std::vector<PromptMessage> messages;
};
```

当调用 `getPrompt("example", {{"arg1","value1"}})` 时，SDK 发送 `prompts/get` 请求，附带参数 `name: "example"` 以及 `arguments: {"arg1":"value1"}`。服务端返回 **GetPromptResult**，其中包含 Prompt 模板实例化后的描述（可选）和消息列表[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。SDK 将 JSON 结果解析为 `PromptContent`：

* `description` 对应结果中的描述（如果有）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* `messages` 对应结果中的 PromptMessage 列表[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    

PromptMessage 在客户端 SDK 中可表示为：

```cpp
struct PromptMessage {
    Role role;              // 枚举或字符串，如 "user" / "assistant"
    Content content;        // 可为 TextContent, ImageContent, EmbeddedResource 等的变体
};
```

SDK 须将 JSON 里的内容类型正确解析到 Content union。例如，如果 `content.type == "text"` 则填充 TextContent结构；如果为 `"resource"`，则构造 EmbeddedResource对象，包含其中的 uri 和文本/二进制数据。

**异步接口** `getPromptAsync` 返回 `std::future<PromptContent>`，方便调用者并行处理。

**使用示例：**

```cpp
// 假设已知服务端有名为 "example-prompt" 的 Prompt，接受参数 "user_input"
PromptContent pc = promptClient.getPrompt("example-prompt", {{"user_input", "Hello"}});
std::cout << "Prompt description: " << pc.description << std::endl;
for(const auto& msg : pc.messages) {
    std::cout << msg.role << ": ";
    if(auto text = std::get_if<TextContent>(&msg.content)) {
        std::cout << text->text;
    } else if(auto res = std::get_if<EmbeddedResource>(&msg.content)) {
        std::cout << "[Embedded Resource: " << res->resource.uri << "]";
    }
    std::cout << std::endl;
}
```

上述流程中，如果 Prompt 包含嵌入资源（例如 msg.content 是 EmbeddedResource），客户端可以选择直接将 `res.resource.text` 提供给模型作为上下文。[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)同时也可利用 `res.resource.uri` 判断是否已缓存该资源内容，以避免重复处理。

### Prompt 列表变更通知 (PromptListChanged Notification)

MCP 协议定义了一个可选的通知 `notifications/prompts/list_changed`，用于服务端在 Prompt 列表发生变化时提醒客户端[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。客户端 SDK 应支持**监听 Prompt 列表更新**事件，例如通过回调机制：

```cpp
class PromptClient {
public:
    using PromptListChangedHandler = std::function<void()>;
    void onPromptListChanged(PromptListChangedHandler handler);
    // ...
};
```

当服务端发送 PromptListChangedNotification 时，SDK 的底层通信层将触发已注册的处理器，通知上层应用可以调用 `listPrompts()` 重新获取最新列表。初期实现中，可选地支持该通知；如不实现，客户端每次启动或按需拉取列表也可满足功能。

### 同步与异步接口的实现考虑

为了兼顾简单易用性和异步性能，客户端接口提供同步和异步两种风格。**同步接口**对于使用方便的场景（如脚本式调用）很有用，而**异步接口**适合集成到事件循环或需要高并发请求的应用中。

在实现上，可以统一使用异步机制，同步接口只是异步的包装。例如：

```cpp
std::vector<Prompt> PromptClient::listPrompts() {
    return listPromptsAsync().get();  // 调用异步接口并等待结果
}
```

这样确保两者行为一致。内部通信可以使用基于事件的网络库（如 Boost.Asio），或平台自带的异步机制。由于 MCP 客户端通常需要处理 Server-Sent Events (SSE) 等流式响应，建议在网络模块中设计统一的异步请求管理。

此外，**错误处理**也应考虑：如请求超时、网络中断或服务器返回错误时，SDK 方法应抛出异常或通过 `std::future` 的异常状态传递给调用者。错误信息可包括 JSON-RPC 的 error 对象内容，以便调试。

## 服务端 SDK 接口设计 (Server SDK Interface)

在服务端，Prompt 模块需要帮助开发者方便地**注册和管理 Prompt 模板**，并处理来自客户端的 Prompt 请求。C++ 服务端 SDK 的设计侧重于：

* 提供数据结构和接口让开发者描述 Prompt 模板（名称、参数、内容模板）。
    
* 处理客户端请求（list 和 get），检索或生成 Prompt 实例并返回。
    
* 与资源子系统集成，在需要时读取资源内容并嵌入。
    
* 支持同步或异步处理，以提升并发性能。
    

下面我们设计服务端 PromptService 及相关组件的接口。

### Prompt 模板定义与注册

服务端应允许开发者在应用初始化时注册可用的 Prompt 模板。可以通过代码注册，或从配置加载。考虑到 C++ 缺乏像 Python 装饰器那样简便的声明方式，SDK 可提供一个**PromptStore**来存储模板，并提供易用的注册函数。例如：

```cpp
class PromptStore {
public:
    bool registerPrompt(const Prompt& promptTemplate);
    bool removePrompt(const std::string& name);
    std::optional<Prompt> findPrompt(const std::string& name) const;
    std::vector<Prompt> listAllPrompts() const;
    // ...
};
```

其中 `Prompt` 为前述定义的模板结构（包含 name, description, arguments）。`registerPrompt` 用于添加一个新的 Prompt 模板，`removePrompt` 则用于动态移除（如果支持的话）。

为方便使用，我们可以在 PromptStore 基础上封装一个 PromptService：

```cpp
class PromptService {
private:
    PromptStore store;
    // 引用或指针：ResourceService, TemplateEngine 等
public:
    // 提供给开发者使用的接口：
    bool addPromptTemplate(const std::string& name, const std::string& description,
                           const std::vector<PromptArgument>& args,
                           const PromptTemplateContent& content);
    bool removePromptTemplate(const std::string& name);

    // 供内部和RPC调用的接口：
    std::vector<Prompt> handleListPrompts();  // 产生 Prompt 列表
    PromptContent handleGetPrompt(const std::string& name, 
                                  const std::map<std::string, std::string>& args);
};
```

这里 `addPromptTemplate` 允许开发者注册一个 Prompt 模板。`PromptTemplateContent` 可以定义为描述模板消息的结构，可以是：

* **静态模板**：直接用一系列 PromptMessage 定义模板内容，其中某些消息的 TextContent 包含占位符（如`{arg1}`）。或者，
    
* **模板字符串**：用一个文本形式包含占位的模板，由 TemplateEngine 来渲染。
    

为了灵活性，我们可以支持以下两种方式：

1. **直接提供 PromptMessage 列表**：开发者自己构造好了模板的消息。例如，一段对话的静态框架。
    
2. **提供模板文本**：开发者提供一个带占位符的字符串，SDK 内部会使用 TemplateEngine 替换。例如：`"Please summarize the following document:\n{{file_content}}"`，将 `{{file_content}}` 替换为参数中的文本内容或嵌入资源。
    

例如，Add Prompt 时：

```cpp
promptService.addPromptTemplate(
    "summarize-file", 
    "Summarize a given text file", 
    { PromptArgument{"file_path", "Path to the text file", true} },
    PromptTemplateContent::fromTemplateString(
        "User: Summarize the content of file {{file_path}}"
    )
);
```

这里 `PromptTemplateContent::fromTemplateString` 可以将字符串保存，并标记需要使用 TemplateEngine 渲染。TemplateEngine 则可使用简易的占位符替换（如 `{{var}}`）。

### Prompt 请求处理流程

**ListPrompts** 请求：当有客户端请求 prompt 列表时（`prompts/list`），PromptService 调用 `handleListPrompts()`：

* 该方法从 PromptStore 中检索所有 Prompt 定义，直接返回 `std::vector<Prompt>`[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。
    
* 如果实现分页（cursor）机制，则根据请求参数决定返回子集及 nextCursor[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。初期可不实现分页（假设 Prompt 数量有限）。
    

**GetPrompt** 请求：当客户端请求获取具体 Prompt 时（`prompts/get`），PromptService 调用 `handleGetPrompt(name, args)`：

1. **查找模板**：通过 PromptStore 找到对应 Prompt 模板。如果不存在则返回错误（可以抛出异常或返回 JSON-RPC error）。
    
2. **模板实例化**：根据存储的 PromptTemplateContent 类型：
    
    * 如果是静态 PromptMessage 列表模板，直接复制列表作为输出的 messages，并跳至下一步替换参数。
        
    * 如果是模板字符串，则调用 TemplateEngine 渲染：将提供的参数字典套用到模板字符串中，生成文本内容。TemplateEngine 应支持简单占位符替换，对于缺失的必需参数返回错误。[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)
        
3. **替换参数**：
    
    * 对于文本参数，直接替换内容中对应部分。例如 PromptMessage 中某字段 `"Please review code: {code}"`，用参数 `"code":"...内容..."` 替换占位符，生成完整文本。
        
    * 对于**资源参数**（如参数值是形如 `file://...` 的 URI），执行资源嵌入逻辑：利用 ResourceService 读取该 URI 内容，然后将 PromptMessage 的 content 从文本替换为 EmbeddedResource：
        
        * 原模板也许写作 `"File content: {{file_content}}"`，TemplateEngine 可特殊处理这种占位：识别参数值是URI且内容需要嵌入，则不直接注入文本，而是生成一个 EmbeddedResource对象。
            
        * 这可通过 MultiModalAssembler/TemplateEngine 与 ResourceService 协作实现：TemplateEngine在渲染时调用 ResourceService 的读取接口获取内容，再通过 MultiModalAssembler封装为 EmbeddedResource PromptMessage。
            
    * 如果 Prompt 模板预先就包含某资源引用（而非参数提供），PromptService 也应调用 ResourceService 获取内容。
        
4. **组装结果**：构造 GetPromptResult 对象，填入可选描述（通常直接使用模板的 description）和最终生成的 PromptMessage 列表[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。例如：
    
    ```cpp
    GetPromptResult result;
    result.description = prompt.description;
    result.messages = generatedMessages;
    ```
    
5. **返回响应**：PromptService 将结果交给底层框架，序列化为 JSON-RPC 应答发回客户端。
    

**资源处理**：在上述第2-3步中，PromptService 需要访问 ResourceService 提供的功能。典型的 ResourceService 接口包括：

```cpp
class ResourceService {
public:
    // 读取资源内容（文本或二进制），返回 pair<data, mimeType>
    ResourceData readResource(const std::string& uri);
    // （可选）获取资源元数据列表
    std::vector<Resource> listResources(const std::string& path);
    // 其他资源管理函数...
};
```

PromptService 应持有 ResourceService 的引用。当需要嵌入资源时：

```cpp
auto data = resourceService.readResource(fileUri);
EmbeddedResource embed;
if(data.isText()) {
    embed.resource = TextResourceContents{ data.text, fileUri, data.mimeType };
} else {
    embed.resource = BlobResourceContents{ data.blob, fileUri, data.mimeType };
}
PromptMessage msg{ Role::User, embed };
messages.push_back(msg);
```

如上，ResourceService::readResource 将处理缓存逻辑，如果缓存有则直接取，否则读取文件/调用API并缓存。这样 PromptService 不需了解缓存细节，只关注获取结果并包装。

### 服务端并发与异步处理

对于服务端 SDK，我们也要考虑**同步/异步**处理模型。简单场景下，同步实现足够：Prompt list/get 请求通常开销小（除非嵌入大文件）。可以在接收到 JSON-RPC 请求时，由线程池中的线程直接调用 PromptService 对应方法并返回结果。

然而，在下列情况下异步是有益的：

* 服务端需要处理大量并发 Prompt 请求，异步 IO 可提高吞吐。
    
* Prompt 实例化涉及等待其他IO（如调用第三方API获取数据作为Prompt的一部分）。
    

为此，PromptService 方法可以设计为同步接口（如上 handleGetPrompt 返回 PromptContent），由上层RPC框架决定是否以异步方式调用。例如集成 Boost.Asio，可将文件读取放在异步任务中执行，再回调组装结果。

初期实现可以采用**多线程并行**模型：每个请求由独立线程处理（例如通过 `std::async` 调用）。在PromptService内部也可以利用异步ResourceService（如ResourceService提供异步read），但复杂度增加。鉴于prompt请求相对快速，初期同步实现较为简单可靠，后续可替换为更复杂的异步架构。

### 接口示例与错误处理

开发者使用服务端 SDK 典型流程：

```cpp
PromptService promptService;
ResourceService resourceService; // 假设已初始化

// 注册Prompt模板
promptService.addPromptTemplate(
    "greet-user",
    "Greet the user by name",
    { PromptArgument{"name", "The user's name", true} },
    PromptTemplateContent::fromTemplateString("Assistant: Hello, {{name}}!")
);

// （Server框架收到 list 请求）
auto promptList = promptService.handleListPrompts(); 
// -> 返回包含 ("greet-user", "Greet the user by name", 参数列表) 的 Prompt 数组

// （Server框架收到 get 请求 name="greet-user", args={"name":"Alice"}）
auto promptResult = promptService.handleGetPrompt("greet-user", {{"name","Alice"}});
// -> PromptResult.description="Greet the user by name"
// -> PromptResult.messages=[ {role:"assistant", content:TextContent("Hello, Alice!")} ]
```

在实现中需注意：

* **错误情况**：如果客户端请求未知的 Prompt 名称，handleGetPrompt 应抛出异常或返回错误，最终转换为 JSON-RPC Error。如 `{"error": {"code": -32602, "message": "Unknown prompt name"}}`。
    
* **参数校验**：必需参数未提供，或提供了未定义的参数名，都应返回错误。[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)
    
* **嵌入资源错误**：如果资源读取失败（文件不存在等），也应返回错误或在Prompt消息中放置一个错误提示（该策略需根据协议设计；通常工具/资源错误可能转化为对话内容提示模型出错）。
    

## 初期内存缓存方案 (Initial In-Memory Caching Strategy)

在没有立即采用数据库的前提下，内存缓存将承担**提高性能**和**减轻资源重复读取**的任务。根据之前资源管理部分的讨论，这里总结初步的内存缓存方案，并为将来引入数据库作准备：

* **Prompt 模板缓存**：PromptStore 本身即缓存了所有 Prompt 模板定义（通常注册后很少变动）。如果支持动态增删 Prompt，可在每次更改时发送 PromptListChanged 通知以使客户端更新[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。Prompt 模板数据量小，可长期存于内存，无需特别持久化（除非希望服务器重启后仍保留动态添加的 Prompt，这需要数据库支持，后续迭代实现）。
    
* **资源内容缓存**：如前述，ResourceService 维护一个内存缓存（例如 `std::unordered_map<std::string, ResourceData>`）。策略包括：
    
    * 缓存键使用资源 URI，使同一资源多次访问命中缓存。
        
    * ResourceData 存储内容（文本或二进制）和元数据（mimeType、大小、最后修改时间等）。
        
    * 采用 LRU 机制限制缓存大小，或者设定总字节上限。例如缓存总大小不超过100 MB，超出时清除最老未使用的项。
        
    * 对于**大文件**，可以考虑只缓存其摘要（如哈希）并在每次需要时读取部分内容。不过在 Prompt 场景下，通常需要全文提供给 LLM，因此倾向缓存全文。
        
    * 当有数据库时，可将资源内容或索引持久化，以便重启后不用重新读取。不过初期实现中，可简单依赖文件系统本身（文件作为真源，每次服务端重启需重新读取，影响不大）。
        
* **线程安全**：由于可能多线程访问缓存，需使用互斥锁保护缓存的数据结构，或采用线程安全容器（C++17的`std::shared_mutex`配合读写锁，以提升读并发）。
    
* **缓存更新失效**：可以为每个 ResourceData 存储一个时间戳或版本号，如果检测到源数据改变则更新缓存。例如对于文件，可记录上次读取时的修改时间，与下一次文件的实际修改时间比对来决定是否刷新。
    

**为数据库支持做准备**：设计缓存时，可将 **ResourceStore** 抽象为接口，提供**内存实现**和**数据库实现**两套。初期用 MemoryResourceStore，实现方法如上；后续可开发例如 SqliteResourceStore，将资源的元数据和内容（或路径）存储在 SQLite 数据库中。因为已经有 `SqlitePromptStore.hpp` 等文件存在，可以推测未来需求包括 Prompt 模板和资源持久化。为平滑过渡：

* 确保缓存接口与数据库接口一致，例如都继承自 `IResourceStore`，支持 get/set/invalidate 等操作。
    
* 数据库接入时，可选择**延迟写入**：先走内存缓存，再异步写入数据库，减少读写延迟。
    
* 提供配置选项切换缓存后端，实现从纯内存升级到数据库的**最小改动**。
    

## 项目结构与模块划分建议 (Project Structure and Module Organization)

为提升代码可维护性和清晰度，我们建议对现有 C++ 项目按功能模块划分目录，并改进命名和序列化代码组织：

### 模块与目录划分

建议采用**分层分模块**的目录结构，例如：

```
src/
├── models/        # 数据模型(struct)定义，如 Prompt, PromptMessage, Resource 等
├── services/      # 业务服务类，实现具体逻辑，如 PromptService, ResourceService 等
├── serialization/ # 序列化与反序列化代码，如 JSON-RPC 请求/响应结构映射
├── transport/     # 通信传输层，例如 SSE 客户端/服务端实现，JsonRpc handler 等
├── client/        # 客户端应用/库相关代码（如果需要单独编译成lib，可隔离）
├── server/        # 服务端应用/启动相关代码
└── utils/         # 通用工具（如日志、配置）等
```

对应的 `include/` 目录也做类似划分。现有结构中，将 **prompt/resource/tools 等按域划分**，在代码规模不大时直观，但随着功能增加可能出现跨域交叉。例如 PromptService 需要用 Resource 模块内容，此时明确的模块层次会更清晰。

**具体调整建议：**

* **models(types)**：集中定义 MCP 协议中的数据结构（可细分多个头文件）。目前项目中 `protocol/McpPromptRequests.hpp` 同时包含请求结构和 Prompt 定义，考虑拆分。例如 `models/Prompt.hpp` 定义 Prompt, PromptArgument, PromptMessage 等类型，`models/Resource.hpp` 定义 Resource, EmbeddedResource 等。这样服务端和客户端都可共享这些模型定义。
    
* **services**：放置 PromptService, ResourceService, ToolService 等实现业务逻辑的类，以及 PromptStore 等存储接口实现。例如 `services/PromptService.hpp/cpp`（处理请求逻辑）、`services/PromptStoreMemory.hpp/cpp`、`services/PromptStoreSqlite.hpp/cpp`。目前 `.include/prompt/PromptService.hpp` 等可归入此目录。`MultiModalAssembler` 和 `TemplateEngine` 作为 PromptService 的辅助组件也放在 prompt 模块目录下，目前合理，也可移至services下作为 PromptService 实现细节（或放 utils）。
    
* **serialization(protocol)**：负责 JSON 与模型对象的转换。可参考当前 `protocol/JsonRpc.hpp`, `McpPromptRequests.hpp` 等，将 JSON-RPC 相关结构和 (de)serialization 函数集中管理。例如 `serialization/JsonRpc.h` 定义基础 JSON-RPC message结构（id, method, params等），`serialization/PromptRpc.hpp` 实现 Prompt 模块请求/响应与模型的转换函数。这样可以将繁杂的 JSON 字段操作与业务逻辑解耦。
    
* **transport**：实现传输层，如 SSE/WebSocket/stdio。当前项目中的 `transport` 目录应包含通信相关类，例如 SSEServer, SSEClient 或 Connection 类等。MCP 支持标准传输如 stdio 和 SSE[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=,protocol%20messages%20and%20lifecycle%20events)[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=from%20mcp)，C++实现可提供如 `StdioTransport`（标准输入输出流处理）和 `SseTransport`（服务器推送事件处理）等。
    
* **client/server**：如果项目需要分别构建客户端和服务端可执行程序，可在各自文件夹提供 main 函数和组装逻辑。例如 `src/client_main.cpp` 使用 PromptClient 连接服务器并演示，`src/server_main.cpp` 初始化 PromptService/ResourceService，侦听端口等。目前看到 repository 中已有 `client_main.cpp` 和 `server_main.cpp`。
    

通过以上划分，实现**模型与逻辑解耦**，**网络与业务解耦**，方便日后替换或重构。例如协议升级只需修改 serialization 层，持久化更换只需在 services 层更改实现。

### JSON 结构绑定代码的优化 (Struct/JSON Binding Code Improvements)

目前的 JSON 绑定可能是**手工编写**解析/序列化代码（通过诸如 `nlohmann::json` 库的 `to_json/from_json` 定义，或手动解析字段）。可以考虑以下改进：

* **使用 JSON 库的自动绑定功能**：如采用 _nlohmann/json_，可利用宏 `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` 定义结构与JSON的互转，无需每个字段手写映射。例如：
    
    ```cpp
    struct Prompt { std::string name; std::string description; std::vector<PromptArgument> arguments; };
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Prompt, name, description, arguments)
    ```
    
    这样json库可自动将 JSON 对象转换为 Prompt 对象[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。对Optional字段可通过 `std::optional` 处理。
    
* **字段命名一致性**：若代码中结构体字段名与 JSON schema 中键名不一致，需要特别处理（比如 JSON 用`camelCase`，C++ 惯用 `snake_case`）。建议统一使用与 JSON 相同的命名以减少转换负担，或在序列化函数中明确指定键名。**例如**，使用 `json["nextCursor"] = obj.next_cursor;` 明确对应。如果大量键需转换命名风格，可写辅助宏或函数集中处理映射。
    
* **代码生成**：考虑到 MCP Schema 较大，也可以借助工具根据 JSON Schema 自动生成C++结构和解析代码。这种方式初期投入较高，但长远可减少人工错误。已有 Rust 和 Dart 等语言的实现可作参考[docs.rs](https://docs.rs/mcp-schema#:~:text=mcp_schema%20,EmbeddedResource%3A%20An%20embedded)[pkg.go.dev](https://pkg.go.dev/github.com/mark3labs/mcp-go/mcp#:~:text=mcp%20,create%20a%20new%20EmbeddedResource)。对于C++，可以使用 like quicktype 之类的工具生成模型类，再手动调整。
    
* **简化 JSON-RPC 封装**：实现一个通用的 JSON-RPC请求/响应基类，包含 `id, method, params or result` 字段[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=%7B%20,%7D)[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=supports)。各具体请求（如 ListPromptsRequest）继承或组合这个基类。这样序列化时可以先序列化通用部分，再处理特定部分。这比每个请求重复拼接 `"jsonrpc": "2.0"` 等字段更清晰。
    
* **减少重复代码**：目前 `McpPromptRequests.hpp`, `McpResourceRequests.hpp` 可能有类似结构的代码（列表请求、结果、通知等）。可以通过模板或宏减少重复。例如定义模板：
    
    ```cpp
    template<typename T>
    struct ListResult { std::vector<T> items; std::string nextCursor; };
    ```
    
    然后具体使用 `using ListPromptsResult = ListResult<Prompt>;`，只需定义 Prompt 的序列化，ListResult 可模板化序列化。
    

通过上述措施，**struct <-> JSON 绑定代码**将更为简洁可靠，也方便在协议微调时快速更新。这样开发者在添加新字段、新请求时，所需改动和测试成本更低。

### 文件命名与组织改进

良好的命名有助于理解模块功能。当前项目中，文件命名有的带 `Mcp` 前缀，有的没有；有的用 `.hpp` 扩展名。改进建议：

* **命名统一**：对于公共API头文件，建议使用清晰的模块前缀或放入对应命名空间下以避免冲突。`McpPromptRequests.hpp` 可以更名或拆分为 `PromptRPC.hpp`（RPC部分）和 `PromptTypes.hpp`（类型定义）等，更直观含义。`PromptService` 等保留类名即文件名。
    
* **去除多余前缀**：如果已经在 `mcp` 命名空间或 `protocol` 目录下，可以省略 `Mcp` 前缀。如 `McpContent.hpp` 改为 `ContentTypes.hpp` 即可表达含义且避免重复“MCP”字样。
    
* **扩展名约定**：`.hpp` 与 `.h` 混用可能令人困惑。可以约定所有C++头文件用 `.hpp`（或 `.h`）统一。源文件也统一 `.cpp`。目前 `.DS_Store` 应从版本库中移除。
    
* **目录组织**：在 include 和 src 目录中，确保对应关系。例如 `include/models/Prompt.hpp` 对应实现 `src/models/Prompt.cpp`（如果需要实现函数）。Service类往往有实现，需在 src 下提供 .cpp，实现文件名与类名一致。
    
* **文档与示例**：可以增加 `README.md` 或 docs 目录来说明各模块用途，方便新读者。尤其对于 Prompt 模块，可简单说明如何使用 SDK 定义 Prompt、启动服务端、调用客户端等步骤（类似 Python SDK 文档中的示例[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40mcp.prompt%28%29%20def%20review_code%28code%3A%20str%29%20,Please%20review%20this%20code%3A%5Cn%5Cn%7Bcode)[github.com](https://github.com/modelcontextprotocol/python-sdk#:~:text=%40mcp.prompt%28%29%20def%20debug_error%28error%3A%20str%29%20,)）。
    

通过整理命名和结构，项目将更具可读性。新的贡献者可以快速找到相关代码，例如所有 Prompt 相关的类型和服务都在 prompt模块/目录或 models/services里明确分开，不会遗漏。序列化逻辑也集中在 protocol或serialization模块，避免散落各处。

## 后续迭代计划与优先级 (Future Iteration Plan and Priorities)

为逐步完善 MCP Prompt 模块在 C++ 实现中的功能，建议按照以下优先级进行后续迭代：

1. **完善基本功能**（高优先级）: 实现 Prompt 列表与获取的完整流程，确保客户端可以正确获取 Prompt 模板和实例结果。包括：
    
    * 完成 PromptService 和 PromptClient 的基本方法，实现 JSON-RPC 请求处理与消息编码解码。
        
    * 支持文本参数替换和资源嵌入的核心逻辑，验证可以嵌入中等大小文本文件内容。
        
    * 建立 ResourceService 的初步实现和内存缓存机制，确保常用资源重复请求性能良好。
        
    * 编写单元测试覆盖 PromptStore（注册/查找）、TemplateEngine（简单模板替换）、ResourceService（读取文件、缓存）等核心组件。
        
2. **持久化支持 PromptStore/ResourceStore**（中优先级）: 引入 SQLite 或轻量数据库：
    
    * 实现 `SqlitePromptStore`（已有文件）以存储 Prompt 模板定义，这样 Prompt 模板可通过配置文件或管理界面动态添加，重启后不丢失。
        
    * 实现 `SqliteResourceStore` 或在资源缓存失效时从数据库查询。对于大型资源，可只存元数据和路径在DB中，实际内容仍从文件读取，或者存储预处理后的文本摘要。
        
    * 设计数据库模式，如 Prompt(name, description)表和 PromptArgument 表等，Resource表记录资源基本信息等。
        
    * 使 PromptService 可以根据配置选择使用 MemoryPromptStore 或 SqlitePromptStore，实现**存储层解耦**。
        
3. **丰富 TemplateEngine 功能**（中优先级）: 当前占位符替换可升级为更强大的模板系统：
    
    * 支持基本的条件或循环（若需要多段动态内容拼接）。
        
    * 支持转义、截断等操作，以防止超长内容直接注入导致模型上下文超限。
        
    * 可考虑兼容常见模板语法（如 Mustache/Handlebars 风格）以降低学习成本。
        
    * 加强对于资源占位符的约定，例如 `{{#resource file_path}}...{{/resource}}` 这样的块表示，可由 TemplateEngine 判断使用 EmbeddedResource 或文本插入。
        
4. **扩展内容类型支持**（中优先级）: 测试并确保 Prompt 消息可以包含 **ImageContent**、**AudioContent** 等：
    
    * 提供在 PromptTemplateContent 中直接引用图像的方式（如通过 URI 指向本地图像文件，EmbeddedResource 会处理为 BlobResourceContents）。
        
    * 确认客户端对这些内容的处理（例如客户端在UI展示图像，同时可能将图像转成描述或embedding提供给模型）。
        
    * 这些类型涉及 MIME 处理，ResourceService 需根据扩展名或内容判断 MIME 类型。
        
5. **并发性能与异步化**（中优先级）: 在基本功能稳定后，分析性能瓶颈，考虑引入异步处理：
    
    * 评估在高并发 Prompt 请求下，目前线程模型是否高效，如有需要，集成 Boost::Asio 实现真正的异步非阻塞 IO（特别是文件IO，在Linux上也可使用 AIO）。
        
    * 将 PromptService 的处理拆分为细粒度任务，如参数替换、文件读取并行执行。
        
    * 确保线程安全，特别在异步模型下 PromptStore/ResourceCache 多线程访问的安全和锁粒度优化。
        
6. **完善通知和边缘功能**（低优先级）: 实现诸如 PromptListChangedNotification 的发送，Prompt 模板热更新：
    
    * 当通过某管理接口新增/删除 Prompt 模板时，PromptService自动广播 PromptListChanged 通知给所有连接的客户端，客户端SDK处理该通知更新本地列表。
        
    * 如果LLM支持，让 Prompt 列表可作为某种“命令”或“提示菜单”展示给用户（这可能超出协议本身，但属于应用层，可在服务端增加获取 prompt 列表的语义）。
        
    * Logging & Monitoring：增加日志输出每次 Prompt 调用，资源读取计时，以便监控性能；统计哪些 Prompt 使用频率高以优化。
        
7. **文档和样例**（持续）: 丰富README或文档，提供示例程序：
    
    * 一个示例服务器程序，注册若干 Prompt（演示文本与资源嵌入），并加载一些本地文件作为资源。
        
    * 一个示例客户端程序，请求 Prompt 并将结果简要打印，验证端到端流程。
        
    * 这些示例有助于新用户快速上手，也便于发现接口是否设计合理。
        

按照以上计划分阶段实施，可先确保核心功能可用，再逐步增强性能和功能。高优的功能让整个框架可基本运行起来，然后通过持久化、模板增强等满足更复杂的使用场景。异步优化和通知属于锦上添花的改进，可等待实际需要时投入。

## 小结 (Summary)

MCP Prompt 模块为 AI 应用提供了**标准化的提示模板机制**，在C++实现中需要充分考虑协议规范和多语言SDK的经验。本文设计的 C++ 客户端/服务端 SDK 界面，涵盖了Prompt模板的定义、列举、获取，以及资源嵌入引用的处理逻辑。我们详述了 Prompt 模块的数据结构（Prompt/PromptMessage 等）[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=,nresources%20from%20the%20MCP%20server)，阐明了如何将**资源内容嵌入** Prompt 消息[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)并通过 URI 进行缓存和引用[github.com](https://github.com/supertomato696/task/blob/main/mcp_prompt_cpp/prompt.json#:~:text=)。在此基础上，客户端SDK提供简洁的同步与异步方法调用，使主机应用便捷地获取 Prompt 列表和内容；服务端SDK则通过 PromptStore 和 PromptService 帮助开发者注册模板并处理请求，内部集成 TemplateEngine 和 ResourceService 完成参数替换和资源加载。我们也针对项目架构提出了模块划分和代码组织的优化方案，包括将模型、服务、序列化、传输分层，以及改进 JSON 绑定的实现方式，以提高代码清晰度和维护性。

通过循序渐进的迭代计划，我们将优先实现核心功能并保证稳定，然后引入持久化存储、异步优化等特性。最终，完善的 C++ Prompt 模块 SDK 将使开发者能够像使用 Python/Java SDK 一样高效地构建 MCP 服务端和客户端，实现**可重用的 Prompt 模板**和**灵活的上下文嵌入**，从而充分发挥 Model Context Protocol 在 AI 集成中的威力[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=values%2C%20standardizing%20the%20way%20LLMs,structured%20and%20manageable%20AI%20applications)[infracloud.io](https://www.infracloud.io/blogs/model-context-protocol-simplifying-llm-integration/#:~:text=,that%20LLMs%20can%20interact%20with)。