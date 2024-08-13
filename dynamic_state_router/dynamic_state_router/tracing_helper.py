import contextlib
import os
from dataclasses import dataclass
from typing import Any, Dict, List, no_type_check

import pyroscope
from opentelemetry import context, trace
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.instrumentation import grpc as grpc_instrumentation
from opentelemetry.sdk.resources import Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import (  # ConsoleSpanExporter,; SimpleSpanProcessor,
    BatchSpanProcessor,
)
from opentelemetry.trace.propagation.tracecontext import TraceContextTextMapPropagator
from pyroscope import otel
from viztracer import VizTracer

localhoststr = "localhost"

otel_rootctx = context.get_current()
first_spans = {}

VIZTRACER_REPORTSDIR = "/home/reachy/viztracer_reports"


def pyroscope_enabled() -> bool:
    return os.environ.get("REACHY_ENABLE_PROFILING_PYROSCOPE") is not None


def viztracer_enabled() -> bool:
    return os.environ.get("REACHY_ENABLE_PROFILING_VIZTRACER") is not None


def otel_spans_enabled() -> bool:
    return os.environ.get("REACHY_ENABLE_OTEL_SPANS") is not None


if viztracer_enabled():
    os.makedirs(VIZTRACER_REPORTSDIR, exist_ok=True)


@dataclass
class DummyContext:
    """Class for keeping track of an item in inventory."""

    trace_id: int
    span_id: float


class DummySpan:
    """
    Used when spans are disabled
    """

    N = 0

    def get_span_context(self) -> DummyContext:
        self.N += 1
        return DummyContext(trace_id=self.N, span_id=self.N)

    def set_attributes(_) -> None:
        pass

    def end(_) -> None:
        pass


class PollenSpan(contextlib.ExitStack):
    @no_type_check
    def __init__(
        self,
        tracer: Any,
        trace_name: Any,
        kind: trace.SpanKind = trace.SpanKind.INTERNAL,
        context: Any = None,
        with_pyroscope: bool = True,
        with_viztracer: bool = False,
        pyroscope_tags: Dict[str, str] = {},
    ) -> None:
        super().__init__()
        self.tracer = tracer
        self.trace_name = trace_name
        self.context = context
        self.kind = kind
        self.with_viztracer = with_viztracer
        self.with_pyroscope = with_pyroscope
        self.pyroscope_tags = pyroscope_tags

    @no_type_check
    def __enter__(self):
        """
        Returns only the opentelemetry Span obj for now
        """
        stack = super().__enter__()
        self.span = self.enter_context(
            self.tracer.start_as_current_span(self.trace_name, kind=self.kind, context=self.context)
            if otel_spans_enabled()
            else contextlib.nullcontext(DummySpan)
        )

        if pyroscope_enabled() and self.with_pyroscope:
            self.pyroscope = self.enter_context(pyroscope.tag_wrapper(self.pyroscope_tags))
        if viztracer_enabled() and self.with_viztracer:
            ctx = self.span.get_span_context()
            self.viztracer = VizTracer(
                # verbose=True,
                output_file=f"{VIZTRACER_REPORTSDIR}/{ctx.trace_id}-{ctx.span_id}-{self.trace_name}.json",
                log_async=True,
                log_gc=True,
            )
        return stack


def tracer(service_name: str, grpc_type: str = "") -> trace.Tracer | None:
    if otel_spans_enabled():
        match grpc_type:
            case "":
                # not auto-instrumentate
                pass
            case "server":
                grpc_instrumentation.GrpcInstrumentorServer().instrument()  # type: ignore
            case "client":
                grpc_instrumentation.GrpcInstrumentorClient().instrument()  # type: ignore
            case _:
                ValueError("Sorry, no numbers below zero")

        # resource = Resource(attributes={"service.name": "grpc_server"})
        resource = Resource(attributes={"service.name": service_name})
        provider = TracerProvider(resource=resource)

        if pyroscope_enabled():
            provider.add_span_processor(otel.PyroscopeSpanProcessor())
        provider.add_span_processor(BatchSpanProcessor(OTLPSpanExporter(endpoint=f"http://{localhoststr}:4317")))

        trace.set_tracer_provider(provider)
        # trace.get_tracer_provider().add_span_processor(
        #     BatchSpanProcessor(OTLPSpanExporter(endpoint="http://localhost:4317"))
        # )

        return trace.get_tracer(service_name)
    return None


def span_links(span: trace.Span, spans: List[trace.Link] = []) -> List[trace.Link]:
    spans.append(trace.Link(span.get_span_context()))
    return spans


def configure_pyroscope(service_name: str, tags: Dict[str, str]) -> None:
    if pyroscope_enabled():
        pyroscope.configure(
            application_name=service_name,  # replace this with some name for your application
            server_address=f"http://{localhoststr}:4040",  # replace this with the address of your Pyroscope server
            sample_rate=5000,  # default is 100
            detect_subprocesses=True,  # detect subprocesses started by the main process; default is False
            oncpu=False,  # report cpu time only; default is True
            gil_only=False,  # only include traces for threads that are holding on to the GIL; default is True
            # enable_logging=True,  # does enable logging facility; default is False
            enable_logging=False,  # does enable logging facility; default is False
            report_pid=True,  # default False
            report_thread_id=True,  # default False
            report_thread_name=True,  # default False
            tags=tags,
        )


def first_span(key: str) -> trace.Span:
    if key not in first_spans:
        first_spans[key] = trace.get_current_span()
        print("first_span key:", key, first_spans[key])
    return first_spans[key]


#######################################################################################
# dummy function to be disabled when otel spans off


def real_travel_span(name: str, start_time: int, tracer: trace.Tracer, context: trace.SpanContext | None = None) -> None:
    """
    Creates a span with a provided start_time.
    This is a workaround to simulate having started the span in the past.
    It's used to create a "fake" span for messages traveling between processes.
    """
    with tracer.start_span(name, start_time=start_time, context=context):  # type: ignore
        pass


def dummy_travel_span(name: str, start_time: int, tracer: None, context: Any = None) -> None:
    pass


TRACEPARENT_STR = "traceparent"


def real_traceparent() -> str:
    carrier: Dict[str, str] = {}
    TraceContextTextMapPropagator().inject(carrier)
    return carrier[TRACEPARENT_STR]


def dummy_traceparent() -> str:
    return ""


def real_ctx_from_traceparent(traceparent: str) -> context.Context:
    carrier = {TRACEPARENT_STR: traceparent}
    return TraceContextTextMapPropagator().extract(carrier=carrier)


def dummy_ctx_from_traceparent(_: str) -> None:
    return None


travel_span = real_travel_span
traceparent = real_traceparent
ctx_from_traceparent = real_ctx_from_traceparent
if not otel_spans_enabled():
    travel_span = dummy_travel_span  # type: ignore
    traceparent = dummy_traceparent
    ctx_from_traceparent = dummy_ctx_from_traceparent  # type: ignore
#######################################################################################
