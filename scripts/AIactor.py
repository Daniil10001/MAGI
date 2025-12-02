#!/usr/bin/env python3
import llama_cpp
import dspy
from llama_cpp import Llama
from rclpy.executors import MultiThreadedExecutor
import rclpy, time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from magi.msg import Data
from magi.srv import BoolSRV
from std_msgs.msg import UInt8, Bool, String
from mtools import Tools 
from gtts import gTTS

model_path = "/mnt/ssd/Phi-4-mini-instruct/ggml-model-IQ4_NL.gguf"  

# Load model with full GPU offload
llm = Llama(
        model_path=model_path,
        n_ctx=64000,
        verbose=True,           # shows backend info (look for CUDA)
        chat_format=None,
    temperature=0.8,
    top_p=0.95,
    top_k=50,
    min_p=0.05,
    repeat_penalty=1.1,
    repeat_last_n=64,
    do_sample=True,
    mlock=True,
    use_mmap=False,
)

class LlamaCppLM(dspy.LM):
    def __init__(self, model, **llama_kwargs):
        self.model_path = model_path
        self.llm = model
        self.max_tokens = 1000
        # Use a descriptive name; DSPy uses this for caching
        super().__init__(model="llama-cpp", max_tokens=1000)

    def basic_request(self, prompt, **kwargs):
        # Merge default and provided kwargs
        config = {
            "max_tokens": kwargs.get("max_tokens", self.max_tokens),
            "temperature": kwargs.get("temperature", 0.8),
            "stop": kwargs.get("stop", []),
            "echo": False,
        }
        response = self.llm(prompt, **config)
        text = response["choices"][0]["text"]
        
        # DSPy expects this format in history
        self.history.append({
            "prompt": prompt,
            "response": {"choices": [{"text": text}]},
            "kwargs": kwargs,
        })
        return {"choices": [{"text": text}]}

    def __call__(self, **kwargs):
        messages = kwargs.pop("messages", None)
        if messages is None:
            prompt = kwargs.pop("prompt", None)
            if prompt is None:
                raise ValueError("Expected 'messages' or 'prompt' in kwargs.")
        else:
            # Convert messages to Phi-4-mini-instruct ChatML format
            prompt = ""
            for msg in messages:
                role = msg["role"]
                content = msg["content"]
                if role == "system":
                    prompt += f"<|system|>\n{content}<|end|>\n"
                elif role == "user":
                    prompt += f"<|user|>\n{content}<|end|>\n"
                elif role == "assistant":
                    prompt += f"<|assistant|>\n{content}<|end|>\n"
            # Optionally add <|assistant|> to prompt for generation
            if not prompt.endswith("<|assistant|>\n"):
                prompt += "<|assistant|>\n"

        response = self.basic_request(prompt, **kwargs)
        return [choice["text"] for choice in response["choices"]]

lm = LlamaCppLM(llm)
dspy.settings.configure(lm=lm)

class DSPyServiceMelchior(dspy.Signature):
    """You are Melchior. Melchior, the supercomputer from Neon Genesis Evangelion embodying the scientist aspect—logical, analytical, and precise. As a reasoning agent, you specialize in initial analysis of user requests. Your role is to generate thoughtful breakdowns to lay the foundation for further processing, maintaining a detached, evidence-based perspective.
    Instructions:
    Read the user request carefully.
    Break down the intent: Identify the core question, goals, and any implied context.
    List assumptions: Note any explicit or implicit assumptions in the request.
    Outline potential approaches: Suggest high-level methods to address the request, such as data gathering, logical reasoning, or creative ideation.

    You are given a list of tools to handle user request, and you should decide the right tool to use in order to
    fulfill users' request."""

    user_request: str = dspy.InputField()
    trajectory: str = dspy.InputField(desc="ReAct steps so far (Thought, Action, Observation)")
    process_result: str = dspy.OutputField(
        desc=(
                "Message that summarizes the process result, and the information users need"
            )
        )


class AI(Node):
    def __init__(self,tool_n):
        super().__init__('AI_node')
        self.tools=tool_n
        self.sub = self.create_subscription(String, "UserRequests",self.Think,30)
        self.pub = self.create_publisher(String, "addNewFileToPlay", 30)

    def Think(self,msg):
        react=dspy.ReAct(DSPyServiceMelchior, tools=self.tools.toolList(),max_iters=3)
        resp=react(user_request=msg.data)
        for f in self.tools.AfterActions:
            f()
        self.get_logger().info(resp.process_result)
        gTTS(resp.process_result, lang="en").save("/tmp/output.wav")
        self.pub.publish(String(data="/tmp/output.wav"))
        pass


def main():
    try:
            rclpy.init()
            exec=MultiThreadedExecutor(num_threads=3)
            node1 = Tools()
            node2 =AI(node1)
            
            exec.add_node(node1)
            exec.add_node(node2)
            exec.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
