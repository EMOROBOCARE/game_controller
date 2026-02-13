import re
import unicodedata
from typing import Iterable, List

import rclpy
from rclpy.node import Node

from chatbot_msgs.srv import EvaluateAnswer, GetResponse, KeywordAction, Rephrase, ResetModel


def _normalize_text(value: str) -> str:
    text = unicodedata.normalize("NFD", str(value or ""))
    text = "".join(char for char in text if unicodedata.category(char) != "Mn")
    text = text.lower()
    text = re.sub(r"[^a-z0-9ñáéíóúü\s]", " ", text)
    text = re.sub(r"\s+", " ", text).strip()
    return text


def _token_set(value: str) -> set:
    normalized = _normalize_text(value)
    return {token for token in normalized.split(" ") if token}


class LLMServiceMockNode(Node):
    def __init__(self) -> None:
        super().__init__("llm_service_mock")
        self.create_service(Rephrase, "/chatbot/rephrase", self._handle_rephrase)
        self.create_service(EvaluateAnswer, "/chatbot/evaluate_answer", self._handle_evaluate_answer)
        self.create_service(GetResponse, "/chatbot/get_response", self._handle_get_response)
        self.create_service(KeywordAction, "/chatbot/keyword_action", self._handle_keyword_action)
        self.create_service(ResetModel, "/chatbot/reset", self._handle_reset)
        self.get_logger().info("LLM mock services ready")

    def _rephrase_candidates(self, sentence: str) -> List[str]:
        source = str(sentence or "").strip()
        if not source:
            return []
        head = source[:1].upper() + source[1:]
        return [
            f"{head}.",
            f"Vamos: {source}",
            f"Intenta esto: {source}",
            f"Puedes decir: {source}",
            f"A ver, {source}",
        ]

    def _is_forbidden(self, value: str, forbidden: Iterable[str]) -> bool:
        low_value = _normalize_text(value)
        if not low_value:
            return True
        for item in forbidden:
            token = _normalize_text(item)
            if token and token in low_value:
                return True
        return False

    def _handle_rephrase(self, request: Rephrase.Request, response: Rephrase.Response):
        sentence = str(request.sentence or "").strip()
        max_alternatives = max(1, int(request.n_alternatives or 1))
        forbidden = list(request.forbidden_expressions or [])
        pool = []
        seen = set()
        for candidate in self._rephrase_candidates(sentence):
            normalized = _normalize_text(candidate)
            if normalized in seen:
                continue
            seen.add(normalized)
            if self._is_forbidden(candidate, forbidden):
                continue
            pool.append(candidate)
            if len(pool) >= max_alternatives:
                break
        if not pool and sentence:
            pool = [sentence]
        response.success = bool(pool)
        response.alternatives = pool
        response.message = "ok" if pool else "empty sentence"
        return response

    def _handle_evaluate_answer(self, request: EvaluateAnswer.Request, response: EvaluateAnswer.Response):
        expected = _normalize_text(request.expected_answer)
        answer = _normalize_text(request.answer)
        is_correct = False
        if expected and answer:
            if expected == answer:
                is_correct = True
            elif len(expected) > 2 and expected in answer:
                is_correct = True
            else:
                expected_tokens = _token_set(expected)
                answer_tokens = _token_set(answer)
                if expected_tokens and answer_tokens:
                    overlap = len(expected_tokens.intersection(answer_tokens))
                    ratio = overlap / max(1, len(expected_tokens))
                    is_correct = ratio >= 0.8
        response.label = "AC" if is_correct else "AI"
        return response

    def _handle_get_response(self, request: GetResponse.Request, response: GetResponse.Response):
        user_input = str(getattr(request, "input", "") or "").strip()
        if user_input:
            response.response = f"He entendido: {user_input}"
        else:
            response.response = "Te escucho."
        response.intents = []
        return response

    def _handle_keyword_action(self, request: KeywordAction.Request, response: KeywordAction.Response):
        action_description = str(request.action_description or "").strip()
        if action_description:
            response.robot_response = f"Entendido, {action_description}."
        else:
            response.robot_response = "Entendido."
        response.error_msg = ""
        return response

    def _handle_reset(self, _request: ResetModel.Request, response: ResetModel.Response):
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LLMServiceMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
