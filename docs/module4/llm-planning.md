# LLM-based Planning

## Introduction to LLM-Based Robot Planning

Large Language Models (LLMs) represent a paradigm shift in robotic planning, enabling robots to interpret natural language commands and generate executable action sequences. This approach bridges the gap between human communication and robotic action, allowing for more intuitive human-robot interaction. In the context of Physical AI and humanoid robotics, LLM-based planning transforms high-level natural language instructions into detailed, executable ROS 2 action sequences.

LLM-based planning systems leverage the vast knowledge and reasoning capabilities of large language models to understand complex commands, reason about the environment, and generate appropriate action plans. This approach enables robots to handle ambiguous instructions, reason about object affordances, and adapt their behavior based on contextual understanding.

## LLM Integration Architecture

### ROS 2 LLM Node Implementation
LLM integration with ROS 2 requires specialized nodes that handle natural language processing and action generation:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
import openai
import json
import asyncio
from typing import Dict, List, Any

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')

        # Command input subscription
        self.command_sub = self.create_subscription(
            String, 'natural_language_command', self.command_callback, 10
        )

        # Action plan publisher
        self.plan_pub = self.create_publisher(String, 'action_plan', 10)

        # Robot state subscription
        self.state_sub = self.create_subscription(
            String, 'robot_state', self.state_callback, 10
        )

        # Configuration parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('llm_model', 'gpt-4'),
                ('api_key', ''),
                ('max_tokens', 1000),
                ('temperature', 0.3),
                ('system_prompt', self.get_default_system_prompt())
            ]
        )

        # Initialize LLM client
        api_key = self.get_parameter('api_key').value
        if api_key:
            openai.api_key = api_key
        else:
            self.get_logger().warn('No API key provided, using mock responses')

        self.current_state = {}
        self.action_library = self.initialize_action_library()

    def get_default_system_prompt(self):
        return """
        You are an AI planning assistant for a humanoid robot. Your role is to convert natural language commands into executable action plans for ROS 2.

        Available actions:
        - navigate_to_location: Move robot to specified location
        - detect_object: Identify objects in the environment
        - grasp_object: Pick up an object
        - place_object: Place an object at a location
        - speak: Generate speech output
        - wait: Pause for specified time

        Respond with a JSON array of action objects, each containing:
        - action_type: The type of action
        - parameters: Required parameters for the action
        - description: Brief description of the action

        Example:
        [
            {
                "action_type": "navigate_to_location",
                "parameters": {"location": "kitchen"},
                "description": "Navigate to the kitchen"
            }
        ]
        """

    def command_callback(self, msg):
        # Process natural language command
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Generate action plan using LLM
        plan = self.generate_action_plan(command, self.current_state)

        if plan:
            # Publish action plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            self.get_logger().info(f'Generated plan: {plan}')

    def generate_action_plan(self, command: str, state: Dict) -> List[Dict]:
        try:
            # Prepare the prompt with context
            prompt = f"Command: {command}\nCurrent state: {state}\n\nGenerate action plan:"

            # Call LLM to generate action plan
            if openai.api_key:
                response = openai.ChatCompletion.create(
                    model=self.get_parameter('llm_model').value,
                    messages=[
                        {"role": "system", "content": self.get_parameter('system_prompt').value},
                        {"role": "user", "content": prompt}
                    ],
                    max_tokens=self.get_parameter('max_tokens').value,
                    temperature=self.get_parameter('temperature').value
                )
                plan_text = response.choices[0].message['content'].strip()
            else:
                # Mock response for testing
                plan_text = '[{"action_type": "speak", "parameters": {"text": "Hello, I received your command."}, "description": "Acknowledge command"}]'

            # Parse the response as JSON
            plan = json.loads(plan_text)
            return plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing LLM response as JSON: {e}')
            return []
        except Exception as e:
            self.get_logger().error(f'Error generating action plan: {e}')
            return []

    def state_callback(self, msg):
        # Update current robot state
        try:
            self.current_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid state message format')
```

### Action Mapping and Execution
Mapping LLM-generated plans to ROS 2 actions:

- **Action Library**: Comprehensive library of available robot actions
- **Parameter Validation**: Ensuring action parameters are valid
- **Execution Sequencing**: Proper sequencing of action execution
- **Error Handling**: Managing action execution failures

## Advanced Planning Capabilities

### Context-Aware Planning
LLM-based planning systems incorporate environmental and situational context:

```python
class ContextAwarePlanningNode(LLMPlanningNode):
    def __init__(self):
        super().__init__()

        # Subscribe to environmental context
        self.vision_sub = self.create_subscription(
            String, 'vision_objects', self.vision_callback, 10
        )
        self.location_sub = self.create_subscription(
            String, 'current_location', self.location_callback, 10
        )
        self.map_sub = self.create_subscription(
            String, 'map_data', self.map_callback, 10
        )

        self.vision_context = {}
        self.location_context = {}
        self.map_context = {}

    def vision_callback(self, msg):
        # Update vision context
        try:
            self.vision_context = json.loads(msg.data)
        except json.JSONDecodeError:
            self.vision_context = {}

    def location_callback(self, msg):
        # Update location context
        self.location_context = {'current_location': msg.data}

    def map_callback(self, msg):
        # Update map context
        try:
            self.map_context = json.loads(msg.data)
        except json.JSONDecodeError:
            self.map_context = {}

    def generate_action_plan(self, command: str, state: Dict) -> List[Dict]:
        # Combine all contexts
        context = {
            'current_state': state,
            'vision_context': self.vision_context,
            'location_context': self.location_context,
            'map_context': self.map_context
        }

        # Prepare enhanced prompt with context
        prompt = f"Command: {command}\nContext: {context}\n\nGenerate action plan:"

        # Generate plan with enhanced context
        return super().generate_action_plan_with_context(command, context)

    def generate_action_plan_with_context(self, command: str, context: Dict) -> List[Dict]:
        # Enhanced system prompt with context awareness
        enhanced_prompt = f"""
        You are an AI planning assistant for a humanoid robot. Consider the following context when generating plans:

        Current State: {context['current_state']}
        Vision Context: {context['vision_context']}
        Location Context: {context['location_context']}
        Map Context: {context['map_context']}

        Generate an appropriate action plan for: {command}
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.get_parameter('llm_model').value,
                messages=[
                    {"role": "system", "content": self.get_parameter('system_prompt').value},
                    {"role": "user", "content": enhanced_prompt}
                ],
                max_tokens=self.get_parameter('max_tokens').value,
                temperature=self.get_parameter('temperature').value
            )
            plan_text = response.choices[0].message['content'].strip()
            return json.loads(plan_text)
        except Exception as e:
            self.get_logger().error(f'Error generating contextual action plan: {e}')
            return []
```

### Multi-Step Task Planning
Complex tasks requiring multiple sequential actions:

- **Task Decomposition**: Breaking complex commands into subtasks
- **Dependency Management**: Managing dependencies between actions
- **Resource Allocation**: Managing robot resources during task execution
- **Plan Refinement**: Adjusting plans based on execution feedback

## Safety and Validation

### Plan Validation and Safety Checks
LLM-generated plans require validation before execution:

```python
class SafeLLMPlanningNode(ContextAwarePlanningNode):
    def __init__(self):
        super().__init__()
        self.safety_validator = SafetyValidator()

    def validate_plan(self, plan: List[Dict]) -> tuple[bool, List[str]]:
        """Validate plan for safety and feasibility"""
        issues = []

        for i, action in enumerate(plan):
            # Check action type validity
            if action['action_type'] not in self.action_library:
                issues.append(f"Invalid action type at step {i}: {action['action_type']}")

            # Check safety constraints
            safety_check = self.safety_validator.check_action(action)
            if not safety_check['safe']:
                issues.append(f"Safety violation at step {i}: {safety_check['reason']}")

            # Check resource availability
            resource_check = self.check_resource_availability(action)
            if not resource_check['available']:
                issues.append(f"Resource conflict at step {i}: {resource_check['reason']}")

        return len(issues) == 0, issues

    def execute_plan_safely(self, plan: List[Dict]):
        """Execute plan with safety monitoring"""
        is_valid, issues = self.validate_plan(plan)

        if not is_valid:
            self.get_logger().error(f'Plan validation failed: {issues}')
            # Generate safety response
            safety_msg = String()
            safety_msg.data = f"Plan rejected due to safety issues: {', '.join(issues)}"
            self.plan_pub.publish(safety_msg)
            return

        # Execute validated plan
        for i, action in enumerate(plan):
            self.get_logger().info(f'Executing action {i+1}/{len(plan)}: {action["description']}")

            # Execute action and monitor for safety
            success = self.execute_single_action(action)

            if not success:
                self.get_logger().error(f'Action execution failed at step {i}')
                break
```

### Human-in-the-Loop Validation
Incorporating human oversight for critical decisions:

- **Confirmation Requests**: Asking for human approval for critical actions
- **Plan Review**: Allowing humans to review and modify generated plans
- **Override Capabilities**: Allowing humans to override LLM decisions
- **Learning from Corrections**: Improving planning based on human feedback

## Integration with ROS 2 Ecosystem

### Action Server Integration
Integrating LLM planning with ROS 2 action servers:

```python
import rclpy.action
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from custom_action_interfaces.action import ExecutePlan

class LLMActionPlanningNode(LLMPlanningNode):
    def __init__(self):
        super().__init__('llm_action_planning_node')

        # Create action server for plan execution
        self.plan_action_server = ActionServer(
            self,
            ExecutePlan,
            'execute_plan',
            self.execute_plan_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """Accept or reject plan execution goal"""
        self.get_logger().info('Received plan execution goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject plan execution cancellation"""
        self.get_logger().info('Received plan execution cancellation')
        return CancelResponse.ACCEPT

    def execute_plan_callback(self, goal_handle):
        """Execute the provided plan"""
        feedback_msg = ExecutePlan.Feedback()
        result = ExecutePlan.Result()

        try:
            # Parse the plan from goal
            plan = json.loads(goal_handle.request.plan_json)

            # Validate the plan
            is_valid, issues = self.validate_plan(plan)
            if not is_valid:
                result.success = False
                result.error_message = f"Plan validation failed: {', '.join(issues)}"
                goal_handle.succeed()
                return result

            # Execute each action in the plan
            for i, action in enumerate(plan):
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.error_message = "Plan execution cancelled"
                    goal_handle.canceled()
                    return result

                # Update feedback
                feedback_msg.current_action = action['description']
                feedback_msg.progress = (i + 1) / len(plan) * 100.0
                goal_handle.publish_feedback(feedback_msg)

                # Execute the action
                success = self.execute_single_action(action)
                if not success:
                    result.success = False
                    result.error_message = f"Action failed: {action['description']}"
                    goal_handle.succeed()
                    return result

            result.success = True
            result.completed_actions = len(plan)
            goal_handle.succeed()

        except Exception as e:
            result.success = False
            result.error_message = f"Plan execution error: {str(e)}"
            goal_handle.succeed()

        return result
```

## Performance and Evaluation

### Planning Quality Metrics
Quantitative assessment of LLM-based planning performance:

- **Plan Success Rate**: Percentage of successfully executed plans
- **Plan Quality**: Assessment of plan optimality and completeness
- **Response Time**: Time from command to plan generation
- **Safety Compliance**: Percentage of plans passing safety validation

### Human-Robot Interaction Quality
Assessment of planning effectiveness in human-robot interaction:

- **Naturalness**: How natural and intuitive the interaction feels
- **Accuracy**: How well the robot executes the intended task
- **Robustness**: How well the system handles ambiguous commands
- **User Satisfaction**: Subjective assessment of the planning system