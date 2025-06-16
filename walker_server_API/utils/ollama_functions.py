import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from langchain_ollama import ChatOllama
from langchain_core.messages import SystemMessage
from langchain.agents import Tool
from utils.agent_tools import activate_assisted_navigation,activate_free_navigation,create_path_for_navigation,start_training_mode
from utils.agent_prompt import control_agent_tool_prompt_template
from langgraph.prebuilt import tools_condition 
from langgraph.prebuilt import ToolNode
from langgraph.graph import StateGraph, MessagesState, START, END

def crient_ollama(name_model):
    client_= ChatOllama(model=name_model, temperature=0.0)
    return client_

#Agent creation:

def agentCreator():
    model = crient_ollama('qwen2.5:14b')
    
    sys_msg = SystemMessage(content= control_agent_tool_prompt_template())
    
    tools = [activate_assisted_navigation,activate_free_navigation,create_path_for_navigation,start_training_mode]
    llm_with_tools = model.bind_tools(tools)
    

    def reasoner(state: MessagesState):
        print("============")
        print("reasoner node")
        print("============")
        messages = state["messages"]
        
        result = llm_with_tools.invoke([sys_msg] + messages)
        print(result)
        return {"messages": result}

    builder =StateGraph(MessagesState)
    builder.add_edge(START, "reasoner")
    builder.add_node("reasoner", reasoner)
    builder.add_node ("tools", ToolNode(tools))
    builder.add_conditional_edges("reasoner",tools_condition,)
    builder.add_edge("tools", "reasoner")
    agent= builder.compile()
  
    
    return agent

def activate_free_navigate(val = False):
    print ("ok")
