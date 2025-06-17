from langgraph.graph import StateGraph, MessagesState, START, END
from langchain_core.tools import tool
from langchain_ollama import ChatOllama
from langchain.callbacks.manager import CallbackManager
from langchain_core.prompts import ChatPromptTemplate
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from langchain_core.messages import HumanMessage, SystemMessage
from langgraph.prebuilt import ToolNode
from typing import List, Literal
from langgraph.prebuilt import tools_condition 
from IPython.display import Image, display


@tool
def decision_function_mode(query: str) -> str:
    """Decides the operating mode of the walker based on the user's input"""
    query = query.lower()
    print(f"Value from llm: {query}")
    if "free navigation" in query or "free mode" in query:
        return "Free navigation"
    elif "assisted navigation" in query or "assisted mode" in query:
        return "Assisted navigation"  
    return "None" 

@tool
def create_path_for_navigation(query: str) -> str:
    """Generates a path for navigation based on the user's input"""
    return f"Caminho gerado para: {query}"  

callback_manager = CallbackManager([StreamingStdOutCallbackHandler()])

# Load the LLM model
LLM_model = ChatOllama(
    model="qwen2.5:32b",
    temperature=0.0,
    num_predict=1024,
    num_ctx=8094,
    callback_manager=callback_manager,
    seed=None,
)

prompt = ChatPromptTemplate.from_messages(
    [
        ("system", """
You are an intelligent assistant controlling a walker. With each interaction with the user, you check to see if there is a need to change the walker’s operating mode. The goal is to continuously identify the user's intent and adjust the navigation mode accordingly. The walker has two operating modes:

- **Free Navigation**: The user walks freely without assistance.
- **Assisted Navigation**: The user wants to be guided or assisted to reach their destination.

Whenever the user makes a request, check for a clear indication that the navigation mode needs to be changed. If so, use the mode decision tool to make the switch.

If the walker is in assisted navigation mode and the user states a destination, you should use the entire user input sentence to find an appropriate path to the destination. Use the path generation tool to calculate the route and guide the user to the requested location.

If the request is for assisted navigation and the user does not indicate a destination, you should ask or wait for more details about the desired location.

Always check all sentences to identify whether the user wants to change the operating mode. If the message is informal, respond in a friendly manner, but always taking into account the required operating mode.
"""),
        ("human", "{input}"),
    ]
)

chain = prompt | LLM_model
tools = [decision_function_mode, create_path_for_navigation]
llm_with_tools = LLM_model.bind_tools(tools)

sys_msg = SystemMessage (content= "You are a useful assistant in charge of controlling the operating mode of a smart walker based on the intentions extracted from the user's sentence. The walker has two operating modes: free navigation (where the user does not need your help to walk) or assisted navigation (where you can assist the user by creating a safe route to the destination point if requested). If during use the user talks to you about subjects other than navigation-related commands, just respond in a friendly manner.")

def reasoner (state: MessagesState):
    
    # query = state["messages"][-1].content if state["messages"] else ""
    messages = state["messages"]

    # sys_msg = SystemMessage(content="You are a helpful assistant in charge of determining whether the user wants to use the walker in assisted navigation or free navigation mode from the phrase spoken by the user and if the navigation mode is assisted, use the tool to find a path for the user to follow.")
    # message = HumanMessage(content=query)
    # messages.append(message)
    # print(f"message lixu: {messages}")
    # print(f"messgae complete: {[sys_msg] + messages}")
    #decision = decision_function_mode(query)
    #state["mode"] = decision 
    result = [llm_with_tools.invoke([sys_msg] + messages)]
    #return {"messages":result}
    """if decision == "Assisted navigation":
        path = create_path_for_navigation(query)
        state["path"] = path  # Save the path to state
        result = [f"Modo de navegação: {state['mode']}", f"Caminho gerado: {path}"]
    elif decision == "Free navigation":
        result = [f"Modo de navegação: {state['mode']}"]"""
    return {"messages": result}

builder =StateGraph (MessagesState)
builder.add_node(reasoner)
builder.add_node ("tools", ToolNode(tools))
builder.add_edge(START, "reasoner")
builder.add_conditional_edges(
    "reasoner",
    # If the latest message (result) from node reasoner is a tool call -> tools_condition routes to tools
    # If the latest message (result) from node reasoner is a not a tool call -> tools_condition routes to END
    tools_condition,
)
builder.add_edge("tools", "reasoner")
agent= builder.compile()

"""display(Image(agent.get_graph(xray=True).draw_mermaid_png()))
graph_image = agent.get_graph(xray=True).draw_mermaid_png()
# Salvar a imagem em um arquivo no diretório desejado
with open("/home/gabriel/Desktop/graph_image2.png", "wb") as f:
    f.write(graph_image)"""

#response = agent.invoke({"messages": [HumanMessage(content="Take me to the nearest cafe")]})
#response['messages'][-1].pretty_print()

# messages = [HumanMessage(content="Can you guide me to Human centered Systems")]
messages = [HumanMessage(content="I am feeling bad, can you do something to help?")]
messages = agent.invoke({"messages": messages})

for m in messages['messages']:
    m.pretty_print()

