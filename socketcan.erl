-module(socketcan).
-export([start/0, stop/0]).
-export([open/1, send/3, recv/1]).

start() ->
  start("socketcan_driver").

start(SharedLib) ->
  case erl_ddll:load_driver("build", SharedLib) of
    ok -> ok;
    {error, already_loaded} -> ok;
    _ -> exit({error, could_not_load_driver})
  end,
  spawn(fun() -> init(SharedLib) end).

init(SharedLib) ->
  register(socketcan_lid, self()),
  Port = open_port({spawn, SharedLib}, [binary]),
  loop(Port).

stop() ->
  socketcan_lid ! stop.

open(Port) -> call_port({open, Port}).
send(Socket, CanID, Data) -> call_port({send, Socket, CanID, Data}).
recv(Socket) -> call_port({recv, Socket}).

call_port(Msg) ->
  socketcan_lid ! {call, self(), Msg},
  receive
    {socketcan_lid, Result} ->
      Result
  end.

loop(Port) ->
  receive
    {call, Caller, Msg} ->
      %Port ! {self(), {command, encode(Msg)}},
      erlang:port_command(Port, term_to_binary(Msg)),
      receive
        {Port, {data, Data}} ->
          Caller ! {socketcan_lid, binary_to_term(Data)}
      end,
      loop(Port);
    stop ->
      Port ! {self(), close},
      receive
        {Port, closed} ->
          exit(normal)
      end;
    {'EXIT', Port, Reason} ->
      io:format("~p ~n", [Reason]),
      exit(port_terminated)
  end.

